
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ManipulatorRequestBus.h"

namespace ROSConDemo
{
    void ManipulatorRequestHandler::ManipulatorSetPosition(const AZ::Vector3 position){
        Call(FN_ManipulatorSetPosition, position);
    }

    AZ::Vector3 ManipulatorRequestHandler::ManipulatorReportError(){
        AZ::Vector3 error;
        CallResult(error, FN_ManipulatorReportError);
        return error;
    }

    int ManipulatorRequestHandler::ManipulatorGetStatus(){
        int state;
        CallResult(state, FN_ManipulatorReportError);
        return state;
    }

    void ManipulatorRequestHandler::ManipulatorRetract(){
        Call(FN_ManipulatorRetract);
    }


    void ManipulatorRequestHandler::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ManipulatorRequestBus>("ManipulatorRequestBus")
                ->Handler<ManipulatorRequestHandler>()
                ->Event("ManipulatorSetPosition", &ManipulatorRequestBus::Events::ManipulatorSetPosition)
                ->Event("ManipulatorReportError", &ManipulatorRequestBus::Events::ManipulatorReportError)
                ->Event("ManipulatorGetStatus", &ManipulatorRequestBus::Events::ManipulatorGetStatus)
                ->Event("ManipulatorRetract", &ManipulatorRequestBus::Events::ManipulatorRetract);

        }
    }

}

