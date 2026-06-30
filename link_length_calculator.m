run("Simulink\Arm_Assembly_DataFile.m");

base = norm(smiData.RigidTransform(3).translation - smiData.RigidTransform(13).translation);
L1 = norm(smiData.RigidTransform(1).translation - smiData.RigidTransform(4).translation);
L2 = norm(smiData.RigidTransform(5).translation - smiData.RigidTransform(2).translation);
L3 = norm(smiData.RigidTransform(6).translation - smiData.RigidTransform(7).translation);
L4 = norm(smiData.RigidTransform(8).translation - smiData.RigidTransform(9).translation);
tip = norm(smiData.RigidTransform(10).translation - smiData.RigidTransform(11).translation - smiData.RigidTransform(12).translation);

save('arm_dimensions.mat', 'base', 'L1', 'L2', 'L3', 'L4', 'tip');