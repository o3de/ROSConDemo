/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace AppleKraken
{
   using Tags = AZStd::vector<AZStd::string>;
   using AppleEvent = Tags;

   const AZStd::string kPickingFailedEventTag = "failed";
   const AZStd::string kPickingAutomatedEventTag = "automated";
} // namespace AppleKraken
