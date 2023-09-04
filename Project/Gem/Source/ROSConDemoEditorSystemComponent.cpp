/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROSConDemoSystemComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROSConDemoEditorSystemComponent.h>

namespace ROSConDemo
{
    void ROSConDemoEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROSConDemoEditorSystemComponent, ROSConDemoSystemComponent>()->Version(0);
        }
    }

    ROSConDemoEditorSystemComponent::ROSConDemoEditorSystemComponent() = default;

    ROSConDemoEditorSystemComponent::~ROSConDemoEditorSystemComponent() = default;

    void ROSConDemoEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        ROSConDemoSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROSConDemoEditorService"));
    }

    void ROSConDemoEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        ROSConDemoSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROSConDemoEditorService"));
    }

    void ROSConDemoEditorSystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2EditorService"));
        ROSConDemoSystemComponent::GetRequiredServices(required);
    }

    void ROSConDemoEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        ROSConDemoSystemComponent::GetDependentServices(dependent);
    }

    void ROSConDemoEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void ROSConDemoEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void ROSConDemoEditorSystemComponent::OnStartPlayInEditor()
    {
        ROSConDemoSystemComponent::Activate();
    }
    void ROSConDemoEditorSystemComponent::OnStopPlayInEditorBegin()
    {
        ROSConDemoSystemComponent::Deactivate();
    }

} // namespace ROS2
