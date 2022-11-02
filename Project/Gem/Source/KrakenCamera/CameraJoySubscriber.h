#pragma once
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/AzFrameworkModule.h>
#include <AzCore/Component/TickBus.h>
#include "ROS2/ROS2Bus.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace AppleKraken
{
    class CameraJoySubscriber
            : public AZ::Component
            , public AZ::TickBus::Handler

    {
    public:
        AZ_COMPONENT(CameraJoySubscriber, "{A730F213-3D80-4F77-8725-2DD2F0901825}", AZ::Component);

        // AZ::Component interface implementation.
        CameraJoySubscriber() = default;
        ~CameraJoySubscriber() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
 
        void OnTick(float deltaTime, AZ::ScriptTimePoint time);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joySubscriber;

        AZStd::string m_joyTopic{"/joy"};
        AZ::Vector2 m_input {0.0, 0.0};

        AZ::Vector2 m_speeds {0.2, 0.2};
        AZ::EntityId m_reachEntity;

        AZStd::array<int,2>m_buttonsX{6,7};
        AZStd::array<int,2>m_buttonsY{4,5};


    };
} // namespace AppleKraken
