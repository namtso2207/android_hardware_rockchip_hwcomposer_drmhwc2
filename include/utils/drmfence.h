/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DRM_FENCE_H_
#define DRM_FENCE_H_
#include "rockchip/utils/drmdebug.h"

#include <android/sync.h>
#include <libsync/sw_sync.h>
#include <fcntl.h>
#include <vector>
#include <queue>

namespace android {
// C++ wrapper class for sync timeline.
class SyncTimeline {
public:
    SyncTimeline(const SyncTimeline &) = delete;
    SyncTimeline& operator=(SyncTimeline&) = delete;
    SyncTimeline() noexcept {
        int fd = sw_sync_timeline_create();
        if (fd == -1)
            return;
        bFdInitialized_ = true;
        iFd_ = fd;
    }
    void destroy() {
        if (bFdInitialized_) {
            close(iFd_);
            iFd_ = -1;
            bFdInitialized_ = false;
        }
    }
    ~SyncTimeline() {
        destroy();
    }
    bool isValid() const {
        if (bFdInitialized_) {
            int status = fcntl(iFd_, F_GETFD, 0);
            if (status >= 0)
                return true;
            else
                return false;
        }
        else {
            return false;
        }
    }
    int getFd() const {
        return iFd_;
    }
    int IncTimeline() {
        return ++iTimelineCnt_;
    }
private:
    int iFd_ = -1;
    bool bFdInitialized_ = false;
    int iTimelineCnt_ = 0;
};

struct SyncPointInfo {
    std::string driverName;
    std::string objectName;
    uint64_t timeStampNs;
    int status; // 1 sig, 0 active, neg is err
};

static int s_fenceCount = 0;
// Wrapper class for sync fence.
class ReleaseFence {
public:
    ReleaseFence(){};
    ~ReleaseFence() {
        destroy();
    }
    ReleaseFence(ReleaseFence &&fence) noexcept {
        if (fence.isValid()) {
            int fd = dup(fence.getFd());
            if (fd == -1)
                return;
            setFd(fd, fence.getSyncTimelineFd(), fence.getName());
        }
    }
    ReleaseFence(const ReleaseFence &fence) noexcept {
        // This is ok, as sync fences are immutable after construction, so a dup
        // is basically the same thing as a copy.
        if (fence.isValid()) {
            int fd = dup(fence.getFd());
            if (fd == -1)
                return;
            setFd(fd, fence.getSyncTimelineFd(), fence.getName());
        }
    }
    ReleaseFence(const int fd) noexcept {
        int status = fcntl(fd, F_GETFD, 0);
        if (status < 0)
            return;
        setFd(fd, -1);
        bFdInitialized_ = true;
    }
    ReleaseFence(const SyncTimeline &timeline,
              int value,
              const char *name = nullptr) noexcept {
        std::string autoName = "allocReleaseFence";
        autoName += s_fenceCount;
        s_fenceCount++;
        int fd = sw_sync_fence_create(timeline.getFd(), name ? name : autoName.c_str(), value);
        if (fd == -1)
            return;
        setFd(fd, timeline.getFd(), name);

    }
    ReleaseFence(const ReleaseFence &a, const ReleaseFence &b, const char *name = nullptr) noexcept {
        std::string autoName = "mergeReleaseFence";
        autoName += s_fenceCount;
        s_fenceCount++;
        int fd = sync_merge(name ? name : autoName.c_str(), a.getFd(), b.getFd());
        if (fd == -1)
            return;
        setFd(fd, -1, name);
    }
    ReleaseFence& operator=(const ReleaseFence &rhs) noexcept {
        destroy();
        if (rhs.isValid()) {
            setFd(dup(rhs.getFd()), rhs.getSyncTimelineFd(), rhs.getName());
        }
        return *this;
    }
    void clearFd() {
        iFd_ = -1;
        bFdInitialized_ = false;
    }
    void destroy() {
        if (isValid()) {
            close(iFd_);
            clearFd();
        }
    }
    bool isValid() const {
        if (bFdInitialized_) {
            int status = fcntl(iFd_, F_GETFD, 0);
            if (status >= 0)
                return true;
            else
                return false;
        }
        else {
            return false;
        }
    }
    int getFd() const {
        return iFd_;
    }
    int getSyncTimelineFd() const {
        return iSyncTimelineFd_;
    }
    std::string getName() const{
        return sName_;
    }
    int wait(int timeout = -1) {
        return sync_wait(iFd_, timeout);
    }
    int signal() {
        if(iSyncTimelineFd_ < 0)
          return -1;
        return sw_sync_timeline_inc(iSyncTimelineFd_, 1);
    }
    std::vector<SyncPointInfo> getInfo() const {
        std::vector<SyncPointInfo> fenceInfo;
        struct sync_file_info *info = sync_file_info(getFd());
        if (!info) {
            return fenceInfo;
        }
        const auto fences = sync_get_fence_info(info);
        for (uint32_t i = 0; i < info->num_fences; i++) {
            fenceInfo.push_back(SyncPointInfo{
                fences[i].driver_name,
                fences[i].obj_name,
                fences[i].timestamp_ns,
                fences[i].status});
        }
        sync_file_info_free(info);
        return fenceInfo;
    }
    int getSize() const {
        return getInfo().size();
    }
    int getSignaledCount() const {
        return countWithStatus(1);
    }
    int getActiveCount() const {
        return countWithStatus(0);
    }
    int getErrorCount() const {
        return countWithStatus(-1);
    }

    std::string dump() const {
        std::string output;
        for (auto &info : getInfo()) {
          output += info.driverName + ":" + info.objectName + ":" + std::to_string(info.timeStampNs) + ":state=" + std::to_string(info.status) + "\n";
        }
        return output;
    }
private:
    void setFd(int fd, int sync_timeline_fd, std::string name = "UnKnow") {
        iFd_ = fd;
        iSyncTimelineFd_ = sync_timeline_fd;
        sName_ = name;
        bFdInitialized_ = true;
    }
    int countWithStatus(int status) const {
        int count = 0;
        for (auto &info : getInfo()) {
            if (info.status == status) {
                count++;
            }
        }
        return count;
    }
    int iFd_ = -1;
    int iSyncTimelineFd_ = -1;
    bool bFdInitialized_ = false;
    std::string sName_;
};

// Wrapper class for sync fence.
class AcquireFence {
public:
    AcquireFence(){};
    AcquireFence(AcquireFence &&fence) noexcept {
        if (fence.isValid()) {
            int fd = dup(fence.getFd());
            setFd(fd);
        }
    }
    AcquireFence(const AcquireFence &fence) noexcept {
        // This is ok, as sync fences are immutable after construction, so a dup
        // is basically the same thing as a copy.
        if (fence.isValid()) {
            int fd = dup(fence.getFd());
            setFd(fd);
        }
    }

    AcquireFence(const int fd) noexcept {
        int status = fcntl(fd, F_GETFD, 0);
        if (status < 0)
            return;
        setFd(fd);
        bFdInitialized_ = true;
    }
    AcquireFence(const AcquireFence &a, const AcquireFence &b, const char *name = nullptr) noexcept {
        std::string autoName = "mergeAcquireFence";
        autoName += s_fenceCount;
        s_fenceCount++;
        int fd = sync_merge(name ? name : autoName.c_str(), a.getFd(), b.getFd());
        if (fd == -1)
            return;
        setFd(fd);
    }

    ~AcquireFence() {
        destroy();
    }

    void destroy() {
        if (isValid()) {
            close(iFd_);
            clearFd();
        }
    }
    bool isValid() const {
        if (bFdInitialized_) {
            int status = fcntl(iFd_, F_GETFD, 0);
            if (status >= 0)
                return true;
            else
                return false;
        }
        else {
            return false;
        }
    }

    AcquireFence& operator=(AcquireFence &rhs) noexcept {
        destroy();
        if (rhs.isValid()) {
            setFd(dup(rhs.getFd()));
        }
        return *this;
    }

    AcquireFence& operator=(AcquireFence &&rhs) noexcept {
        destroy();
        if (rhs.isValid()) {
            setFd(dup(rhs.getFd()));
        }
        return *this;
    }
    int getFd() const {
        return iFd_;
    }
    int wait(int timeout = -1) {
        return sync_wait(iFd_, timeout);
    }
    std::vector<SyncPointInfo> getInfo() const {
        std::vector<SyncPointInfo> fenceInfo;
        struct sync_file_info *info = sync_file_info(getFd());
        if (!info) {
            return fenceInfo;
        }
        const auto fences = sync_get_fence_info(info);
        for (uint32_t i = 0; i < info->num_fences; i++) {
            fenceInfo.push_back(SyncPointInfo{
                fences[i].driver_name,
                fences[i].obj_name,
                fences[i].timestamp_ns,
                fences[i].status});
        }
        sync_file_info_free(info);
        return fenceInfo;
    }
    int getSize() const {
        return getInfo().size();
    }
    int getSignaledCount() const {
        return countWithStatus(1);
    }
    int getActiveCount() const {
        return countWithStatus(0);
    }
    int getErrorCount() const {
        return countWithStatus(-1);
    }
private:
    void setFd(int fd) {
        iFd_ = fd;
        bFdInitialized_ = true;
    }
    void clearFd() {
        iFd_ = -1;
        bFdInitialized_ = false;
    }
    int countWithStatus(int status) const {
        int count = 0;
        for (auto &info : getInfo()) {
            if (info.status == status) {
                count++;
            }
        }
        return count;
    }
    int iFd_ = -1;
    bool bFdInitialized_ = false;
};
// The semantics of the fences returned by the device differ between
// hwc1.set() and hwc2.present(). Read hwcomposer.h and hwcomposer2.h
// for more information.
//
// Release fences in hwc1 are obtained on set() for a frame n and signaled
// when the layer buffer is not needed for read operations anymore
// (typically on frame n+1). In HWC2, release fences are obtained with a
// special call after present() for frame n. These fences signal
// on frame n: More specifically, the fence for a given buffer provided in
// frame n will signal when the prior buffer is no longer required.
//
// A retire fence (HWC1) is signaled when a composition is replaced
// on the panel whereas a present fence (HWC2) is signaled when a
// composition starts to be displayed on a panel.
//
// The HWC2to1Adapter emulates the new fence semantics for a frame
// n by returning the fence from frame n-1. For frame 0, the adapter
// returns NO_FENCE.
class DeferredRetireFence {
    public:
        DeferredRetireFence()
          : mFences({ReleaseFence(), ReleaseFence()}) {}

        void add(ReleaseFence &&rf) {
            mFences.push(std::move(rf));
            mFences.pop();
        }

        const ReleaseFence &get() const {
            return mFences.front();
        }

        const ReleaseFence &get_back() const {
            return mFences.back();
        }
    private:
        // There are always two fences in this queue.
        std::queue<ReleaseFence> mFences;
};
} // namespace android
#endif