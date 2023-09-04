/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <QtCore/qglobal.h>
#include <ROSConDemoModuleInterface.h>
#include <ROSConDemoEditorSystemComponent.h>

namespace ROSConDemo
{
    class ROSConDemoEditorModule : public ROSConDemoModuleInterface
    {
    public:
        AZ_RTTI(ROSConDemoEditorModule, "{ca41105c-1c8c-1db7-72fe-c20cd0160c76}", ROSConDemoModuleInterface);
        AZ_CLASS_ALLOCATOR(ROSConDemoEditorModule, AZ::SystemAllocator);

        ROSConDemoEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROSConDemoEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROSConDemoEditorSystemComponent>(),
            };
        }
    };
} // namespace ROSConDemo

AZ_DECLARE_MODULE_CLASS(Gem_ROSConDemo, ROSConDemo::ROSConDemoEditorModule)
