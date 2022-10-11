local DynamicAppleSpawner = {
	Properties = {
		Debug  = true,
		DebugColliders=false,
		DestAppleTreeTag="AppleTree",
		FollowTag="FollowTarget",
	 	ApplesPrefab = { default=SpawnableScriptAssetRef(), description="Prefab to spawn"},
	 	NumPrefabsToSpawn = 6,
	 	TranslationThreshold=1, -- distance in meters follow target must move before we update
	 	DetectionRadius=4,
	 	GroundHeight=0
	},
	DestTagHandler = {},
	FollowTagHandler = {}
}

function DynamicAppleSpawner:OnActivate()
 	self.numPrefabsSpawned = 0
	self.appleTrees = {}
	self.appleTreePositions = nil
	self.followTargets = nil
	self.freeAppleGroups = {}
 	self.lastPos = Vector3(0,0,0)
 	self.update = false
 	self.closestAppleTrees = {}
 	self.numPrefabsToSpawn = 0 -- don't spawn prefabs until a follow target exists
 	self.spawningPrefabs = false
 		
	self.appleTreeTag = Crc32(self.Properties.DestAppleTreeTag)
	
	self.DestTagHandler.component = self
	self.DestTagHandler.handler = TagGlobalNotificationBus.Connect(self.DestTagHandler, Crc32(self.Properties.DestAppleTreeTag))
	self.FollowTagHandler.component = self
	self.FollowTagHandler.handler = TagGlobalNotificationBus.Connect(self.FollowTagHandler, Crc32(self.Properties.FollowTag))

	self.spawnableMediator = SpawnableScriptMediator()
	self.ticket = self.spawnableMediator:CreateSpawnTicket(self.Properties.ApplesPrefab)
	self.spawnableNotificationsBusHandler = SpawnableScriptNotificationsBus.Connect(self, self.ticket:GetId())
 	
 	self.tickHandler = TickBus.Connect(self,0)
end

function DynamicAppleSpawner:OnSpawn(spawnTicket, entityList)
	-- first entity should be the root
	table.insert(self.freeAppleGroups, entityList[1])
	--self:SetPhysicsEnabled(entityList[1], false)
	local entityName  = GameEntityContextRequestBus.Broadcast.GetEntityName(entityList[1])
	Debug.Log("$5 Spawned " .. tostring(entityName))
	
	if self.numPrefabsSpawned >= self.numPrefabsToSpawn then
		self.spawningPrefabs = false
	end 
	
	-- print out locations of every apple for scripting purposes
	--local children = TransformBus.Event.GetChildren(entityList[1])
	--for i=1,#children do
	--	local childId = children[i]
	--	local pos = TransformBus.Event.GetLocalTranslation(childId)
	--	Debug.Log("("..tostring(pos.x)..","..tostring(pos.y)..","..tostring(pos.z).."),")
	--end
	
	-- mark all follow targets to update in case they are waiting on groups
	for k,followTarget in pairs(self.followTargets) do
		followTarget.update = true
	end
end

function DynamicAppleSpawner.DestTagHandler:OnEntityTagAdded(entityId)
	table.insert(self.component.appleTrees, entityId)
end

function DynamicAppleSpawner:AddFollowTarget(entityId)
	if self.followTargets == nil then
		self.followTargets = {}
	end
	
	self.followTargets[entityId] = { 
		position=Vector3(0,0,0), 
		update=false, 
		handler = nil,
		OnTransformChanged = function(followTarget, localTM, worldTM) 
			local translation = worldTM:GetTranslation()
			translation.z = self.Properties.GroundHeight
			
			-- only set this target to update if we moved over the threshold
			if followTarget.update == false and followTarget.position:GetDistanceSq(translation) > self.Properties.TranslationThreshold then
				followTarget.update = true
				followTarget.position = translation
				followTarget.position.z = self.Properties.GroundHeight
			end
		 end  
		 }

	self.followTargets[entityId].handler  = TransformNotificationBus.Connect(self.followTargets[entityId], entityId)
	
	self.numPrefabsToSpawn = self.numPrefabsToSpawn + self.Properties.NumPrefabsToSpawn

	local entityName  = GameEntityContextRequestBus.Broadcast.GetEntityName(entityId)
	Debug.Log("$5 Follow target added " .. tostring(entityName))
end

function DynamicAppleSpawner.FollowTagHandler:OnEntityTagAdded(entityId)
	self.component:AddFollowTarget(entityId)
end

function DynamicAppleSpawner:SetPhysicsEnabled(entityId, enabled)
	-- change the physics settings on all descendents
	local descendants = TransformBus.Event.GetAllDescendants(entityId)
	if enabled then
		for i=1,#descendants do
			local descendantEntityId = descendants[i]
			SimulatedBodyComponentRequestBus.Event.DisablePhysics(descendantEntityId)
		end
	else
		for i=1,#descendants do
			local descendantEntityId = descendants[i]
			SimulatedBodyComponentRequestBus.Event.EnablePhysics(descendantEntityId)
		end
	end
end

 function getAppleTreeIndex(appleTrees, entityId)
	for k,tree in ipairs(appleTrees) do
		if tree.entityId == entityId then
			return k
		end
	end
	return -1
end

function hitEntityId(hitArray, entityId)
	for i=1,#hitArray  do
		if hitArray[i].EntityId == entityId then
			return true
		end
	end
	return false
end

function DynamicAppleSpawner:OnTick(delaTime, scriptTime)
	local physicsSystem = GetPhysicsSystem()
	local sceneHandle = physicsSystem:GetSceneHandle(DefaultPhysicsSceneName)
	local scene = physicsSystem:GetScene(sceneHandle)
	local needMorePrefabs = false
	
	for k,followTarget in pairs(self.followTargets) do
		if  followTarget.update then
			followTarget.update = false
			local tm = Transform.CreateTranslation(followTarget.position)
			-- use a small height to avoid intersecting apples if possible
			-- otherwise need to filter
			local dimensions = Vector3(self.Properties.DetectionRadius, self.Properties.DetectionRadius, 0.1)
			local overlapRequest = CreateBoxOverlapRequest(dimensions, tm)			
			local hits = scene:QueryScene(overlapRequest)

			local treesToRemove = {}
			local treesToAdd = {}
			local closestTrees = {}
			
			if hits.HitArray :Size()> 0 then
				if self.Properties.Debug then
					local aabb = Aabb.CreateCenterHalfExtents(followTarget.position, dimensions * 0.5)
					DebugDrawRequestBus.Broadcast.DrawAabb(aabb, Color(1.0,1.0,0.0), 1.0)
					Debug.Log("hit " .. tostring(hits.HitArray:Size()) .. " objects")
				end
				
				for k,tree in ipairs(self.closestAppleTrees) do
					if hitEntityId(hits.HitArray, tree.entityId) == false then
						if self.Properties.Debug then
							Debug.Log("Removing apple group entity " ..tostring(tree.applesEntityId))
						end
						RenderMeshComponentRequestBus.Event.SetVisibility(tree.entityId, true)
						
						-- remove this tree and free the apple group
						if tree.applesEntityId ~= -1 then
							table.insert(self.freeAppleGroups, tree.applesEntityId)
							--self:SetPhysicsEnabled(tree.applesEntityId, false)
							TransformBus.Event.SetWorldTranslation(tree.applesEntityId, Vector3(0,0,-10 * k))
						end
						table.insert(treesToRemove, tree.entityId)
					end
				end
				
				for i=1,#hits.HitArray do
					local entityId = hits.HitArray[i].EntityId
					local index = getAppleTreeIndex(self.closestAppleTrees, entityId)
					
					if index ~= -1 and self.closestAppleTrees[index].applesEntityId ~= -1 then
						table.insert(closestTrees, self.closestAppleTrees[index])
					elseif TagComponentRequestBus.Event.HasTag(entityId, self.appleTreeTag) then
						local applesEntityId = -1
						if #self.freeAppleGroups > 0 then
							applesEntityId = self.freeAppleGroups[#self.freeAppleGroups]
							if applesEntityId == nil or type(applesEntityId) == "number" then
								Debug.Log("Non EntityId found in freeAppleGroups " ..tostring(applesEntityId) .. " contains " ..#self.freeAppleGroups.." items")
								followTarget.update = true
								needMorePrefabs =  true
							else
								if self.Properties.Debug then
									Debug.Log("Claiming apple group " ..tostring(applesEntityId))
								end
								local treeTM = TransformBus.Event.GetWorldTM(entityId)
								--self:SetPhysicsEnabled(applesEntityId, false)
								TransformBus.Event.SetWorldTM(applesEntityId, treeTM)
								--self:SetPhysicsEnabled(applesEntityId, true)
								
								-- hide the render meshes
								RenderMeshComponentRequestBus.Event.SetVisibility(entityId, false)
								table.remove(self.freeAppleGroups)
								table.insert(closestTrees, {entityId=entityId, applesEntityId=applesEntityId})		
								table.insert(treesToAdd, entityId)
							end
						else 
							if self.Properties.Debug then
								Debug.Log("$4 Couldn't find free apple tree group in free pool of size" .. tostring(#self.freeAppleGroups))
							end
							needMorePrefabs =  true
							followTarget.update = true
						end
					end
				end
			else
				for k,tree in ipairs(self.closestAppleTrees) do
					table.insert(treesToRemove, tree.entityId)
				end
			end
			
			
			if self.Properties.Debug then
				for k,tree in ipairs(closestTrees) do
					DebugDrawRequestBus.Broadcast.DrawSphereOnEntity(tree.entityId, 0.5, Color(1.0,0.0,0.0), 1.0)
				end
			end
			self.closestAppleTrees = closestTrees
		end
	end

	if needMorePrefabs and  self.spawningPrefabs  == false then
		self.numPrefabsToSpawn = self.numPrefabsToSpawn + 1
	end
	
	-- spawn one apple group per frame for now
	if self.numPrefabsSpawned < self.numPrefabsToSpawn  then
		self:SpawnAppleGroup()
	end
end

function DynamicAppleSpawner:SpawnAppleGroup()
	self.numPrefabsSpawned = self.numPrefabsSpawned + 1
	
	local tm = TransformBus.Event.GetWorldTM(self.entityId)
	local translation = tm:GetTranslation()
	
	-- move the prefabs down so they don't collide with eachother (safer than disabling/enabling physics)
	translation.z = translation.z - self.numPrefabsSpawned * 10
	local rotation = tm:GetRotation():GetEulerRadians()

	Debug.Log("$7 Spawning apple group " .. tostring(self.numPrefabsSpawned ))
	self.spawningPrefabs = true
	self.spawnableMediator:SpawnAndParentAndTransform(self.ticket, self.entityId, translation, rotation, 1.0)
end

function DynamicAppleSpawner:OnDeactivate()
	if self.tickHandler ~= nil then
		self.tickHandler:Disconnect()
		self.tickHandler = nil
	end
	self.appleTrees = nil
	
	self.spawnableNotificationsBusHandler:Disconnect()    
	            
	self.DestTagHandler.handler:Disconnect()
	self.DestTagHandler.handler = nil
	self.FollowTagHandler.handler:Disconnect()
	self.FollowTagHandler.handler = nil
	
	for k,followTarget in pairs(self.followTargets) do
		followTarget.handler:Disconnect()
	end
end

return DynamicAppleSpawner