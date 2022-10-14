----------------------------------------------------------------------------------------------------
--
-- Copyright (c) Contributors to the Open 3D Engine Project.
-- For complete copyright and license terms please see the LICENSE at the root of this distribution.
--
-- SPDX-License-Identifier: Apache-2.0 OR MIT
--
--
--
----------------------------------------------------------------------------------------------------
local DynamicAppleSpawner = {
    Properties = {
        Debug                = true,
        DebugColliders       = false,
        DestAppleTreeTag     = "AppleTree",
        FollowTag            = "FollowTarget",
        ApplesPrefab         = { default = SpawnableScriptAssetRef(), description = "Prefab to spawn" },
        NumPrefabsToSpawn    = 6,
        TranslationThreshold = { default = 1, description = "distance in meters follow target must move before we update" },
        DetectionRadius      = 4,
        GroundHeight         = 0
    },
    FollowTagHandler = {}
}

function DynamicAppleSpawner:OnActivate()
    self.numPrefabsToSpawn = 0 -- don't spawn prefabs until a follow target exists
    self.numPrefabsSpawned = 0
    self.closestAppleTrees = {}
    self.followTargets = {}
    self.freeAppleGroups = {}
    self.spawningPrefabs = false

    local physicsSystem = GetPhysicsSystem()
    local sceneHandle = physicsSystem:GetSceneHandle(DefaultPhysicsSceneName)
    self.scene = physicsSystem:GetScene(sceneHandle)

    self.appleTreeTag = Crc32(self.Properties.DestAppleTreeTag)

    self.FollowTagHandler.component = self
    self.FollowTagHandler.handler = TagGlobalNotificationBus.Connect(self.FollowTagHandler, Crc32(self.Properties.FollowTag))

    self.spawnableMediator = SpawnableScriptMediator()
    self.ticket = self.spawnableMediator:CreateSpawnTicket(self.Properties.ApplesPrefab)
    self.spawnableNotificationsBusHandler = SpawnableScriptNotificationsBus.Connect(self, self.ticket:GetId())

    self.tickHandler = TickBus.Connect(self, 0)
end

function DynamicAppleSpawner:PrintApplePositions(entityId)
    -- print out locations of every apple for scripting (Blender Python) purposes
    local children = TransformBus.Event.GetChildren(entityId)
    for i = 1, #children do
        local childId = children[i]
        local pos = TransformBus.Event.GetLocalTranslation(childId)
        Debug.Log("(" .. tostring(pos.x) .. "," .. tostring(pos.y) .. "," .. tostring(pos.z) .. "),")
    end
end

function DynamicAppleSpawner:OnSpawn(spawnTicket, entityList)
    -- first entity should be the root
    table.insert(self.freeAppleGroups, entityList[1])
    local entityName = GameEntityContextRequestBus.Broadcast.GetEntityName(entityList[1])
    Debug.Log("$5 Spawned " .. tostring(entityName))

    if self.numPrefabsSpawned >= self.numPrefabsToSpawn then
        self.spawningPrefabs = false
    end

    -- mark all follow targets to update in case they are waiting on groups
    for k, followTarget in pairs(self.followTargets) do
        followTarget.update = true
    end
end

function DynamicAppleSpawner:AddFollowTarget(entityId)
    -- add a follow target that we will spawn apples around
    self.followTargets[tostring(entityId)] = {
        position = Vector3(0, 0, 0),
        update = false,
        handler = nil,
        OnTransformChanged = function(followTarget, localTM, worldTM)
            local translation = worldTM:GetTranslation()
            translation.z = self.Properties.GroundHeight

            -- only set this target to update if we moved over the threshold distance
            if followTarget.update == false and
                followTarget.position:GetDistanceSq(translation) > self.Properties.TranslationThreshold then
                followTarget.update = true
                followTarget.position = translation
                followTarget.position.z = self.Properties.GroundHeight
            end
        end
    }

    self.followTargets[tostring(entityId)].handler = TransformNotificationBus.Connect(self.followTargets[tostring(entityId)], entityId)

    self.numPrefabsToSpawn = self.numPrefabsToSpawn + self.Properties.NumPrefabsToSpawn

    local entityName = GameEntityContextRequestBus.Broadcast.GetEntityName(entityId)
    Debug.Log("$5 Follow target added " .. tostring(entityName))
end

function DynamicAppleSpawner.FollowTagHandler:OnEntityTagAdded(entityId)
    self.component:AddFollowTarget(entityId)
end

function DynamicAppleSpawner:SetDescendantVisibility(entityId, visible)
    local descendants = TransformBus.Event.GetAllDescendants(entityId)
    for i = 1, #descendants do
        local descendantEntityId = descendants[i]
        RenderMeshComponentRequestBus.Event.SetVisibility(descendantEntityId, visible)
    end
end

function DynamicAppleSpawner:SetPhysicsEnabled(entityId, enabled)
    -- change the physics settings on all descendents
    -- currently not used because does not work reliably.  colliders do not appear
    -- in correct position after movement
    local descendants = TransformBus.Event.GetAllDescendants(entityId)
    if enabled then
        for i = 1, #descendants do
            local descendantEntityId = descendants[i]
            SimulatedBodyComponentRequestBus.Event.DisablePhysics(descendantEntityId)
        end
    else
        for i = 1, #descendants do
            local descendantEntityId = descendants[i]
            SimulatedBodyComponentRequestBus.Event.EnablePhysics(descendantEntityId)
        end
    end
end

local function getAppleTreeIndex(appleTrees, entityId)
    for k, tree in ipairs(appleTrees) do
        if tree.entityId == entityId then
            return k
        end
    end
    return -1
end

function DynamicAppleSpawner:HaveFollowTargetsToUpdate()
    for k, followTarget in pairs(self.followTargets) do
        if followTarget.update then
            return true
        end
    end

    return false
end

function DynamicAppleSpawner:SetFollowTargetsUpdateStatus(update)
    for k, followTarget in pairs(self.followTargets) do
        followTarget.update = update
    end
end

function DynamicAppleSpawner:GetTreesNearFollowTargets()
    local treesNearFollowTargets = {}
    for k, followTarget in pairs(self.followTargets) do
        local tm = Transform.CreateTranslation(followTarget.position)
        -- use a small box height to avoid intersecting apples if possible
        -- otherwise need to filter out apples
        local dimensions = Vector3(self.Properties.DetectionRadius, self.Properties.DetectionRadius, 0.1)
        local overlapRequest = CreateBoxOverlapRequest(dimensions, tm)
        local hits = self.scene:QueryScene(overlapRequest)
        if hits.HitArray:Size() > 0 then
            for i = 1, #hits.HitArray do
                local entityId = hits.HitArray[i].EntityId
                if TagComponentRequestBus.Event.HasTag(entityId, self.appleTreeTag) then
                    treesNearFollowTargets[tostring(entityId)] = entityId
                    --Debug.Log("Tree found near target " .. tostring(entityId))
                end
            end
        end
    end

    return treesNearFollowTargets
end

function DynamicAppleSpawner:RemoveTreesOutsideFollowTargetRange(treesNearFollowTargets)
    for k, tree in ipairs(self.closestAppleTrees) do
        if treesNearFollowTargets[tostring(tree.entityId)] == nil then
            if self.Properties.Debug then
                Debug.Log("Removing apple group entity " .. tostring(tree.applesEntityId) .. " tree no longer near target " .. tostring(tree.entityId))
            end

            -- reveal the static mesh
            RenderMeshComponentRequestBus.Event.SetVisibility(tree.entityId, true)

            -- remove this tree and free the apple group to be re-used
            if tree.applesEntityId ~= -1 then
                table.insert(self.freeAppleGroups, tree.applesEntityId)
                TransformBus.Event.SetWorldTranslation(tree.applesEntityId, Vector3(0, 0, -10 * k))
            end
        end
    end
end

function DynamicAppleSpawner:ReUseAppleGroup(applesEntityId, treeEntityId)
    if self.Properties.Debug then
        Debug.Log("Claiming apple group " .. tostring(applesEntityId))
    end

    local treeTM = TransformBus.Event.GetWorldTM(treeEntityId)
    if treeTM ~= nil then
        TransformBus.Event.SetWorldTM(applesEntityId, treeTM)
    end

    -- make sure all apples are visible
    self:SetDescendantVisibility(applesEntityId, true)

    -- hide static render meshes
    RenderMeshComponentRequestBus.Event.SetVisibility(treeEntityId, false)

    table.remove(self.freeAppleGroups)
end

function DynamicAppleSpawner:OnTick(delaTime, scriptTime)
    local needMorePrefabs = false

    if self:HaveFollowTargetsToUpdate() then
        local treesNearFollowTargets = self:GetTreesNearFollowTargets()
        self:RemoveTreesOutsideFollowTargetRange(treesNearFollowTargets)
        self:SetFollowTargetsUpdateStatus(false)
        
        local closestTrees = {}
        -- re-use apple groups or spawn new ones
        for k, treeEntityId in pairs(treesNearFollowTargets) do
            local index = getAppleTreeIndex(self.closestAppleTrees, treeEntityId)
            if index ~= -1 and self.closestAppleTrees[index].applesEntityId ~= -1 then
                -- tree is already being tracked
                table.insert(closestTrees, self.closestAppleTrees[index])
            else
                -- tree has the required tag
                local applesEntityId = -1
                if #self.freeAppleGroups > 0 then
                    applesEntityId = self.freeAppleGroups[#self.freeAppleGroups]
                    if applesEntityId == nil or type(applesEntityId) == "number" then
                        Debug.Log("Non EntityId found in freeAppleGroups " ..
                            tostring(applesEntityId) .. " contains " .. #self.freeAppleGroups .. " items")
                        needMorePrefabs = true
                    else
                        self:ReUseAppleGroup(applesEntityId, treeEntityId)
                        table.insert(closestTrees, { entityId = treeEntityId, applesEntityId = applesEntityId })
                    end
                else
                    if self.Properties.Debug then
                        Debug.Log("$4 Couldn't find free apple tree group in free pool of size " .. tostring(#self.freeAppleGroups))
                    end
                    needMorePrefabs = true
                end
            end
        end

        if self.Properties.Debug then
            for k, tree in ipairs(closestTrees) do
                DebugDrawRequestBus.Broadcast.DrawSphereOnEntity(tree.entityId, 0.5, Color(1.0, 0.0, 0.0), 1.0)
            end
        end

        self.closestAppleTrees = closestTrees
    end

    if needMorePrefabs then
    	    self:SetFollowTargetsUpdateStatus(true)
    end
    -- spawn additional apple group prefabs on demand
    if needMorePrefabs and self.spawningPrefabs == false then
        self.numPrefabsToSpawn = self.numPrefabsToSpawn + 1
    end

    -- spawn one apple group per frame
    if self.numPrefabsSpawned < self.numPrefabsToSpawn then
        self:SpawnAppleGroup()
    end
end

function DynamicAppleSpawner:SpawnAppleGroup()
    self.numPrefabsSpawned = self.numPrefabsSpawned + 1

    local tm = TransformBus.Event.GetWorldTM(self.entityId)
    local translation = tm:GetTranslation()

    -- move the prefabs down so they don't collide with eachother (more reliable than disabling/enabling physics)
    translation.z = translation.z - self.numPrefabsSpawned * 10
    local rotation = tm:GetRotation():GetEulerRadians()

    Debug.Log("$7 Spawning apple group " .. tostring(self.numPrefabsSpawned))
    self.spawningPrefabs = true
    self.spawnableMediator:SpawnAndParentAndTransform(self.ticket, self.entityId, translation, rotation, 1.0)
end

function DynamicAppleSpawner:OnDeactivate()
    if self.tickHandler ~= nil then
        self.tickHandler:Disconnect()
        self.tickHandler = nil
    end

    self.spawnableNotificationsBusHandler:Disconnect()

    self.FollowTagHandler.handler:Disconnect()
    self.FollowTagHandler.handler = nil

    for k, followTarget in pairs(self.followTargets) do
        followTarget.handler:Disconnect()
    end
end

return DynamicAppleSpawner
