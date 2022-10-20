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
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>

namespace AppleKraken
{

    //! Experimental, demo only class for simple camera movement smoothing
    // TODO - research damping and smoothing camera, use exponential mapping for rotation smoothing as well.
    class FollowingCameraComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AzFramework::InputChannelEventListener
    {
    public:
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void Reflect(AZ::ReflectContext* reflection);

        AZ_COMPONENT(FollowingCameraComponent, "{92317883-9956-455E-9A1C-BF8986DC2F80}", AZ::Component);

        // AZ::Component
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AzFramework::InputChannelEventListener
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

    private:
        void OnKeyboardEvent(const AzFramework::InputChannel& inputChannel);

        AZ::Vector3 SmoothTranslation() const;

        bool m_isActive = true;

        AZ::Transform m_initialPose;

        AZ::EntityId m_target;

        float m_rotationChange = 0.0f;
        float m_rotationChange2 = 0.0f;

        float m_zoomChange = 0.0f;

        int m_smoothingBuffer = 30;

        float m_zoomSpeed = 0.06f;
        float m_rotationSpeed = 0.05f;

        float m_min_camera_distance = 1.5f;
        float m_max_camera_distance = 25.0f;

        bool m_look_at_center_entity = false;

        bool m_activated = false;

        AZStd::deque<AZStd::pair<AZ::Vector3, float>> m_lastTransforms;
    };
} // namespace AppleKraken
