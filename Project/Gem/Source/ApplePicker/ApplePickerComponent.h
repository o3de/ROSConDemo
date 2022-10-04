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
#include <AzCore/Component/TickBus.h>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace AppleKraken
{
    using TriggerRequest = std::shared_ptr<std_srvs::srv::Trigger::Request>;
    using TriggerResponse = std::shared_ptr<std_srvs::srv::Trigger::Response>;

    //! Demo component handling orchestration of apple picking
    class ApplePickerComponent
        : public AZ::Component
        , public ApplePickingNotificationBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ApplePickerComponent, "{E9E83A4A-31A4-4E7A-AF88-7565AC8B9F27}", AZ::Component);
        ApplePickerComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Detect and pick all apples in manipulator range.
        void StartAutomatedOperation();

        //! Report overall progress of gathering task.
        //! @returns how much of the task is complete (0: nothing, 1: all of it). The task is completed when all reachable apples are
        //! gathered (or had a failure) and the effector is in IDLE state.
        float ReportProgress();

    private:
        void Activate() override;
        void Deactivate() override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void EffectorReadyForPicking() override;
        void ApplePicked() override;
        void AppleRetrieved() override;
        void PickingFailed(const AZStd::string& reason) override;

        bool IsBusy() const;
        void PickNextApple();
        void QueryEnvironmentForAllApplesInBox(const AZ::Obb& globalBox);
        void ProcessTriggerServiceCall(const TriggerRequest req, TriggerResponse resp);

        AZStd::string m_triggerServiceTopic = "trigger_apple_gathering";
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_triggerService;

        AZ::EntityId m_effectorEntityId;
        AZ::Obb m_gatheringArea;
        size_t m_initialTasksSize = 0;
        AZStd::queue<PickAppleTask>
            m_currentAppleTasks; //! Populated in StartAutomatedOperation. Tasks are popped when completed or failed.
    };
} // namespace AppleKraken
