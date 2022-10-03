/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ApplePickingNotifications.h"
#include "ApplePickingRequests.h"
#include <AzCore/Component/Component.h>
// #include <vision_msgs/msgs/detection_3d_array.h>
#include <AzCore/Component/TickBus.h>
namespace AppleKraken
{
    //! Demo component handling orchestration of apple picking
    class ApplePickerComponent
        : public AZ::Component
        , private ApplePickingNotificationBus::Handler
        , private AZ::TickBus::Handler

    {
    public:
        AZ_COMPONENT(ApplePickerComponent, "{E9E83A4A-31A4-4E7A-AF88-7565AC8B9F27}", AZ::Component);
        ApplePickerComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

        //! Detect and pick all apples in manipulator range.
        void StartAutomatedOperation();

        //! Report overall progress of gathering task.
        //! @returns how much of the task is complete (0: nothing, 1: all of it). The task is completed when all reachable apples are
        //! gathered (or had a failure) and the effector is in IDLE state.
        float ReportProgress();

    private:
        void ApplePicked() override;
        void AppleRetrieved() override;
        void PickingFailed(const AZStd::string& reason) override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        AZ::Obb m_gatheringArea;
        AZStd::queue<PickAppleTask> m_currentAppleTasks; //! Populated in StartAutomatedOperation. Tasks are popped when completed or failed.
        int m_field {0};
    };
} // namespace AppleKraken
