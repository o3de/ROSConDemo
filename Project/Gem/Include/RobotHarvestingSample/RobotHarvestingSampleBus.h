/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RobotHarvestingSample
{
    class RobotHarvestingSampleRequests
    {
    public:
        AZ_RTTI(RobotHarvestingSampleRequests, "{2D614DC7-0E0B-4EC9-B0EB-8F2ED827310A}");
        virtual ~RobotHarvestingSampleRequests() = default;

        //! Reload the current level
        virtual void ReloadLevel() = 0;
    };

    class RobotHarvestingSampleBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RobotHarvestingSampleRequestBus = AZ::EBus<RobotHarvestingSampleRequests, RobotHarvestingSampleBusTraits>;
    using RobotHarvestingSampleInterface = AZ::Interface<RobotHarvestingSampleRequests>;

} // namespace RobotHarvestingSample
