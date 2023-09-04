/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROSConDemoSystemComponent.h>

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace ROSConDemo
{
    /// System component for ROS2 editor
    class ROSConDemoEditorSystemComponent
        : public ROSConDemoSystemComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {

    public:
        AZ_COMPONENT(ROSConDemoEditorSystemComponent, "{96e4d699-8ebf-f2cc-9c6f-7791d697e1fa}", ROSConDemoSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        ROSConDemoEditorSystemComponent();
        ~ROSConDemoEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // EditorEntityContextNotificationBus overrides
        void OnStartPlayInEditor() override;
        void OnStopPlayInEditorBegin() override;
        //////////////////////////////////////////////////////////////////////////
    };
} // namespace ROSConDemo
