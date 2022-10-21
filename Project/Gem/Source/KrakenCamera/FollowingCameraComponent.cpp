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
                ->Field("Target", &FollowingCameraComponent::m_target)
                ->Field("SmoothingLength", &FollowingCameraComponent::m_smoothingBuffer)
                ->Field("ZoomSpeed", &FollowingCameraComponent::m_zoomSpeed)
                ->Field("RotationSpeed", &FollowingCameraComponent::m_rotationSpeed);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<FollowingCameraComponent>("Following Camera", "Camera following kraken")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &FollowingCameraComponent::m_isActive, "Active", "")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &FollowingCameraComponent::m_target, "Target", "Entity of the followed object")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FollowingCameraComponent::m_smoothingBuffer, "SmoothingLength", "Number of past transform used to smooth")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FollowingCameraComponent::m_zoomSpeed, "Zoom Speed", "Zoom speed")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FollowingCameraComponent::m_rotationSpeed, "Rotation Speed", "Rotation Speed");
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
    }

    void FollowingCameraComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputChannelEventListener::Disconnect();
    }

    void FollowingCameraComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        if (!m_target.IsValid()){
            AZ_Warning("FollowingCameraComponent", false, "m_target is empty!");
        }
        if (!m_isActive)
        {
            return;
        }

        AZ::Transform target_world_transform;
        EBUS_EVENT_ID_RESULT(target_world_transform, m_target, AZ::TransformBus, GetWorldTM);

        m_lastTransforms.push_back(AZStd::make_pair(target_world_transform.GetTranslation(), deltaTime));
        m_lastRotations.push_back(AZStd::make_pair(target_world_transform.GetRotation(), deltaTime));

        if (m_lastTransforms.size() > m_smoothingBuffer){
            m_lastTransforms.pop_front();
            m_lastRotations.pop_front();
        }
        AZ::Vector3 translation = SmoothTranslation();
        AZ::Quaternion quat = SmoothRotation();
        AZ::Transform filtered_transform =
            {
                translation,
                AZ::Quaternion::CreateRotationZ(quat.GetEulerRadians().GetZ()),
                target_world_transform.GetUniformScale()
            };

        auto modified_transform = m_initialPose.GetInverse();
        AZ::Transform rotation_transform =
            {
                {0.0, 0.0, 0.0},
               AZ::Quaternion::CreateRotationY(m_rotationChange2) *  AZ::Quaternion::CreateRotationZ(m_rotationChange) ,
                1.0
            };
        modified_transform *= rotation_transform;
        modified_transform.Invert();

        AZ::Vector3 translation_modifier = modified_transform.GetBasisY() * m_zoomChange;
        auto modified_translation = modified_transform.GetTranslation() + translation_modifier;
        modified_transform.SetTranslation( modified_translation );

        EBUS_EVENT_ID(GetEntityId(), AZ::TransformBus, SetWorldTM, filtered_transform * modified_transform);


    }

    AZ::Vector3 FollowingCameraComponent::SmoothTranslation() const
    {
        AZ::Vector3 sum{0};
        float normalization{0};
        for (const auto& p : m_lastTransforms){
            sum += p.first * p.second;
            normalization+=p.second;
        }
        return sum/normalization;
    }

    AZ::Quaternion FollowingCameraComponent::SmoothRotation() const
    {
        // Smoothing is done by taking an exponential map. Note that the exponential map is the same thing as 'ScaledAxisAngle'
        // for 3D rotations. This map can be treated as linear space locally.
        // In that linear space the average is computed. It should give a pleasant experience for slow-moving objects.
        float normalization{0};
        AZ::Vector3 sum{0};
        for (const auto& p : m_lastRotations){
            sum += p.second * (p.first.ConvertToScaledAxisAngle());
            normalization+=p.second;
        }
        return AZ::Quaternion::CreateFromScaledAxisAngle(sum/normalization);
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
        // TODO take into account the refresh rate on key pressed for smooth experience
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericW)
        {
            m_zoomChange = AZStd::min(m_zoomMax, m_zoomChange + m_zoomSpeed);
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericA)
        {
            m_rotationChange += m_rotationSpeed;
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericQ)
        {
            m_rotationChange2 += m_rotationSpeed;
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericS)
        {
            m_zoomChange = AZStd::max(m_zoomMin, m_zoomChange - m_zoomSpeed);
        }

        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericD)
        {
            m_rotationChange -= m_rotationSpeed;
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericE)
        {
            m_rotationChange2 -= m_rotationSpeed;
        }

    }
}
