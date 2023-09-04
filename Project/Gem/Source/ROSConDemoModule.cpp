/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */


#include "ROSConDemoModuleInterface.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <ROS2/ROS2Bus.h>


namespace ROSConDemo
{
    class ROSConDemoModule : public ROSConDemoModuleInterface
    {
    public:
        AZ_RTTI(ROSConDemoModule, "{E38575E4-7D2F-4617-B938-416E8C1C07B4}", ROSConDemoModuleInterface);
        AZ_CLASS_ALLOCATOR(ROSConDemoModule, AZ::SystemAllocator, 0);
    };
} // namespace ROSConDemo

AZ_DECLARE_MODULE_CLASS(Gem_ROSConDemo, ROSConDemo::ROSConDemoModule)
