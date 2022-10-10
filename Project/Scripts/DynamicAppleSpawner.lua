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
end

function DynamicAppleSpawner.DestTagHandler:OnEntityTagAdded(entityId)
	table.insert(self.component.appleTrees, entityId)
end

function DynamicAppleSpawner.FollowTagHandler:OnEntityTagAdded(entityId)
	if self.component.followTargets == nil then
		self.component.followTargets = {}
	end
	table.insert(self.component.followTargets, entityId)
	local entityName  = GameEntityContextRequestBus.Broadcast.GetEntityName(entityId)
	Debug.Log("$5 Follow target entity: " .. tostring(entityName))
end

function DynamicAppleSpawner:OnTransformChanged(localTM, worldTM)
	local translation = worldTM:GetTranslation()
	translation.z = self.Properties.GroundHeight
	
	if self.Properties.DebugColliders then
		local pos = worldTM:GetTranslation() + worldTM:GetBasisY() *3.0
		local tm = Transform.CreateTranslation(pos)
		local overlapRequest = CreateSphereOverlapRequest( 0.5, tm)
		local physicsSystem = GetPhysicsSystem()
		local sceneHandle = physicsSystem:GetSceneHandle(DefaultPhysicsSceneName)
		local scene = physicsSystem:GetScene(sceneHandle)
		hits = scene:QueryScene(overlapRequest)
		
		DebugDrawRequestBus.Broadcast.DrawSphereAtLocation(pos, 0.5, Color(0.0,0.0,1.0), 0.2)
		
		if #hits.HitArray > 1 then
			Debug.Log("NumSphereHits " ..tostring(#hits.HitArray))
		end
	end
		
	if self.update == false and self.lastPos:GetDistanceSq(translation) > self.Properties.TranslationThreshold then
		self.update = true
		self.lastPos = translation

		self.lastPos.z = self.Properties.GroundHeight
	end
end

function DynamicAppleSpawner:OnTick(delaTime, scriptTime)

	if self.update then
		self.update = false
		local tm = Transform.CreateTranslation(self.lastPos)
		-- use a small height to avoid intersecting apples if possible
		-- otherwise need to filter
		local dimensions = Vector3(self.Properties.DetectionRadius, self.Properties.DetectionRadius, 0.1)
		local overlapRequest = CreateBoxOverlapRequest(dimensions, tm)
		
		local physicsSystem = GetPhysicsSystem()
		local sceneHandle = physicsSystem:GetSceneHandle(DefaultPhysicsSceneName)
		local scene = physicsSystem:GetScene(sceneHandle)
		hits = scene:QueryScene(overlapRequest)
		local treesToRemove = {}
		local treesToAdd = {}
		local closestTrees = {}
		
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
		
		if hits.HitArray :Size()> 0 then
		
			if self.Properties.Debug then
				local aabb = Aabb.CreateCenterHalfExtents(self.lastPos, dimensions * 0.5)
				DebugDrawRequestBus.Broadcast.DrawAabb(aabb, Color(1.0,1.0,0.0), 1.0)
				Debug.Log("hit " .. tostring(hits.HitArray:Size()) .. " objects")
			end
			
			for k,tree in ipairs(self.closestAppleTrees) do
				if hitEntityId(hits.HitArray, tree.entityId) == false then
					if self.Properties.Debug then
						Debug.Log("Removing apple group entity " ..tostring(tree.applesEntityId))
					end
					-- remove this tree and free the apple group
					if tree.applesEntityId ~= -1 then
						table.insert(self.freeAppleGroups, tree.applesEntityId)
						TransformBus.Event.SetWorldTranslation(tree.applesEntityId, Vector3(0,0,-100))
					end
					table.insert(treesToRemove, tree.entityId)
				end
			end
			
			
			for i=1,#hits.HitArray do
				local entityId = hits.HitArray[i].EntityId
				local index = getAppleTreeIndex(self.closestAppleTrees, entityId)
				
				if index ~= -1 then
					table.insert(closestTrees, self.closestAppleTrees[index])
				elseif TagComponentRequestBus.Event.HasTag(entityId, self.appleTreeTag) then
					local applesEntityId = -1
					if #self.freeAppleGroups > 0 then
						applesEntityId = self.freeAppleGroups[#self.freeAppleGroups]
						if applesEntityId == nil or type(applesEntityId) == "number" then
							Debug.Log("Non EntityId found in freeAppleGroups " ..tostring(applesEntityId) .. " contains " ..#self.freeAppleGroups.." items")
						else
							if self.Properties.Debug then
								Debug.Log("Claiming apple group " ..tostring(applesEntityId))
							end
							local treeTM = TransformBus.Event.GetWorldTM(entityId)
							TransformBus.Event.SetWorldTM(applesEntityId, treeTM)
							table.remove(self.freeAppleGroups)
						end
					elseif self.Properties.Debug then
						Debug.Log("$4 Couldn't find free apple tree group containing" .. tostring(#self.freeAppleGroups) .. " elements")
					end
					table.insert(closestTrees, {entityId=entityId, applesEntityId=applesEntityId})
					table.insert(treesToAdd, entityId)
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
	
	if self.followTargets ~= nil then
		self.transformHandler = TransformNotificationBus.Connect(self, self.followTargets[1])
	end
	
	-- get apple tree positions
	if self.appleTreePositions == nil and false then
		local positions = {}
		self.appleTreePositions = {}
		for k,entityId in ipairs(self.appleTrees) do
			local tm = TransformBus.Event.GetWorldTM(entityId)
			local position = tm:GetTranslation()
			table.insert(positions, {entityId=entityId, transform=tm,  position2D=Vector2(position.x, position.y)})
			
			self.appleTreePositions[entityId] = {transform=tm, closest={}}
			--table.insert(self.appleTreePositions, {EntityId=entityId, Transform=tm, Closest={}})
		end
		
		local numClosest = 16
		-- do a one time distance analysis (slow, but better than runtime check)
		for k,appleTree in ipairs(appleTreePositions) do
			local translation = appleTree.tm:GetTranslation()
			local position2D = Vector2(translation.x, translation.y)
			
			table.sort(positions, function (a,b)  return position2D:GetDistanceSq(a.position2D) < position2D:GetDistanceSq(b.position2D) end)
			
			for j,otherPosition in ipairs(positions) do
				table.insert(appleTree.closest, otherPosition.entityId)
			end
		end
	end
	
	
	-- spawn apples in pool
	if self.numPrefabsSpawned < self.Properties.NumPrefabsToSpawn  then
		self.numPrefabsSpawned = self.numPrefabsSpawned + 1
		Debug.Log("$7 Spawning apple group " .. tostring(self.numPrefabsSpawned ))
		local tm = TransformBus.Event.GetWorldTM(self.entityId)
		local translation = tm:GetTranslation()
		local rotation = tm:GetRotation():GetEulerRadians()
		self.spawnableMediator:SpawnAndParentAndTransform(self.ticket, self.entityId, translation, rotation, 1.0)
	end
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
end

return DynamicAppleSpawner