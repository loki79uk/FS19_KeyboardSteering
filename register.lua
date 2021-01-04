-- ============================================================= --
-- KEYBOARD STEERING MOD
-- ============================================================= --
KeyboardSteeringREGISTER = {};

g_specializationManager:addSpecialization('keyboardSteering', 'KeyboardSteering', Utils.getFilename('KeyboardSteering.lua', g_currentModDirectory), true);

for name, data in pairs( g_vehicleTypeManager:getVehicleTypes() ) do
	local vehicleType = g_vehicleTypeManager:getVehicleTypeByName(tostring(name));
	if  SpecializationUtil.hasSpecialization(Drivable,  data.specializations) and
		SpecializationUtil.hasSpecialization(Motorized, data.specializations) and
		SpecializationUtil.hasSpecialization(Enterable, data.specializations) and
		not SpecializationUtil.hasSpecialization(Locomotive, data.specializations) and
		not SpecializationUtil.hasSpecialization(ConveyorBelt, data.specializations) then
			--print(tostring(vehicleType.name))
			g_vehicleTypeManager:addSpecialization(name, 'keyboardSteering')
	end
end
