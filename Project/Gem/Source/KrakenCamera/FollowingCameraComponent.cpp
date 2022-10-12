/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "FollowingCameraComponent.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>
#include <MathConversion.h>


namespace AppleKraken
{

    void FollowingCameraComponent::Reflect(AZ::ReflectContext* reflection)
    {
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(reflection);
        if (serializeContext)
        {
            serializeContext->Class<FollowingCameraComponent, AZ::Component>()
                ->Version(1)
                ->Field("Is active", &FollowingCameraComponent::m_isActive)
                ->Field("Target", &FollowingCameraComponent::m_target);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<FollowingCameraComponent>("Following Camera", "Camera following kraken")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &FollowingCameraComponent::m_isActive, "Active", "")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &FollowingCameraComponent::m_target, "Target", "Entity of the followed object");
            }
        }
    }

    void FollowingCameraComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
    }

    void FollowingCameraComponent::Init()
    {
    }

    void FollowingCameraComponent::Activate()
    {
        InputChannelEventListener::Connect();
        AZ::TickBus::Handler::BusConnect();

        EBUS_EVENT_ID_RESULT(m_initialPose, GetEntityId(), AZ::TransformBus, GetLocalTM);

        m_observedXYCoords.fill(std::nullopt);
    }

    void FollowingCameraComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputChannelEventListener::Disconnect();
    }

    void FollowingCameraComponent::OnTick(float /*deltaTime*/, AZ::ScriptTimePoint /*time*/)
    {
        if (!m_isActive)
        {
            return;
        }

        AZ::Transform target_world_transform;
        EBUS_EVENT_ID_RESULT(target_world_transform, m_target, AZ::TransformBus, GetWorldTM);
        
        m_observedXYCoords[m_ticksCounter] = {target_world_transform.GetTranslation().GetX(), target_world_transform.GetTranslation().GetY()};
        auto xy_avgs = ObservedXYAverage();
        
        AZ::Transform filtered_transform =
            {
                AZ::Vector3{xy_avgs.first, xy_avgs.second, 0.0},
                AZ::Quaternion::CreateRotationZ(target_world_transform.GetEulerRadians().GetZ()),
                target_world_transform.GetUniformScale()
            };

        auto modified_transform = m_initialPose.GetInverse();
        AZ::Transform rotation_transform =
            {
                {0.0, 0.0, 0.0},
                AZ::Quaternion::CreateRotationZ(m_rotationChange),
                1.0
            };
        modified_transform *= rotation_transform;
        modified_transform.Invert();

        AZ::Vector3 translation_modifier = modified_transform.GetBasisY() * m_zoomChange;
        auto modified_translation = modified_transform.GetTranslation() + translation_modifier;
        modified_transform.SetTranslation( modified_translation );

        EBUS_EVENT_ID(GetEntityId(), AZ::TransformBus, SetWorldTM, filtered_transform * modified_transform);

        m_ticksCounter++;
        m_ticksCounter = m_ticksCounter % CAMERA_FILTER_BUFFER_SIZE;
    }

    std::pair<float, float> FollowingCameraComponent::ObservedXYAverage() const
    {
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        int valid_counter = 0;

        for (auto xy : m_observedXYCoords)
        {
            if (xy == std::nullopt)
            {
                break;
            }
            valid_counter++;
            sum_x += xy->first;
            sum_y += xy->second;
        }

        return std::make_pair(sum_x / valid_counter,sum_y / valid_counter);
    }

    bool FollowingCameraComponent::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        if (!m_isActive)
        {
            return false;
        }

        const AzFramework::InputDeviceId& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();

        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId))
        {
            OnKeyboardEvent(inputChannel);
        }

        return false;
    }

    void FollowingCameraComponent::OnKeyboardEvent(const AzFramework::InputChannel& inputChannel)
    {
        if (!m_isActive)
        {
            return;
        }

        const AzFramework::InputChannelId& channelId = inputChannel.GetInputChannelId();

        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericW)
        {
            // magic number which blocks user from zooming in too close
            if (m_zoomChange < 3)
                m_zoomChange += 0.06;
        }

        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericA)
        {

            m_rotationChange += 0.02;
        }

        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericS)
        {
            // magic number which blocks user from zooming out too far
            if (m_zoomChange > -25)
                m_zoomChange -= 0.06;
        }

        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericD)
        {
            m_rotationChange -= 0.02;
        }
    }
}
