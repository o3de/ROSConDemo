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
#include <AzFramework/Input/Events/InputChannelEventListener.h>
#include <AzFramework/Components/TransformComponent.h>

namespace AppleKraken
{
    static constexpr int CAMERA_FILTER_BUFFER_SIZE = 30;

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

        [[nodiscard]] std::pair<float, float> ObservedXYAverage() const;

        bool m_isActive = true;

        AZ::Transform m_initialPose;

        AZ::EntityId m_target;

        float m_rotationChange = 0.0f;

        float m_zoomChange = 0.0f;

        int m_ticksCounter = 0;

        std::array<std::optional<std::pair<float, float>>, CAMERA_FILTER_BUFFER_SIZE> m_observedXYCoords = {};
    };
}
