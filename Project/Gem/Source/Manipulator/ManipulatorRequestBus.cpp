
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ManipulatorRequestBus.h"

namespace AppleKraken
{
    void ManipulatorRequestHandler::PickApple(const AZ::Vector3 position)
    {
        Call(FN_PickApple, position);
    }

    AZ::Vector3 ManipulatorRequestHandler::GetPosition()
    {
        AZ::Vector3 p;
        CallResult(p, FN_GetPosition);
        return p;
    }

    void ManipulatorRequestHandler::Retrieve()
    {
        Call(FN_Retrieve);
    }

    void ManipulatorRequestHandler::RetrieveNose()
    {
        Call(FN_RetrieveNose);
    }
    int ManipulatorRequestHandler::GetStatus()
    {
        int p;
        CallResult(p, FN_GetStatus);
        return p;
    }
    bool ManipulatorRequestHandler::IsNoseRetreived()
    {
        bool p;
        CallResult(p, FN_IsNoseRetreived);
        return p;
    }


    void ManipulatorRequestHandler::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ManipulatorRequestBus>("ManipulatorRequestBus")
                ->Handler<ManipulatorRequestHandler>()
                ->Event("PickApple", &ManipulatorRequestBus::Events::PickApple)
                ->Event("GetPosition", &ManipulatorRequestBus::Events::GetPosition)
                ->Event("Retrieve", &ManipulatorRequestBus::Events::Retrieve)
                ->Event("GetStatus", &ManipulatorRequestBus::Events::GetStatus)
                ->Event("IsNoseRetreived", &ManipulatorRequestBus::Events::IsNoseRetreived)
                ->Event("RetrieveNose", &ManipulatorRequestBus::Events::RetrieveNose);
        }
    }

} // namespace AppleKraken
