/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "GatheringRowRequests.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/vector.h>

namespace AppleKraken
{
    using GatheringPoses = AZStd::vector<AZ::Transform>;

    //! Demo component wrapping information about gathering positions (single row of apple trees)
    class GatheringRowRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        virtual GatheringPoses GetGatheringPoses() const = 0;
    };

    using GatheringRowRequestBus = AZ::EBus<GatheringRowRequests>;
} // namespace AppleKraken
