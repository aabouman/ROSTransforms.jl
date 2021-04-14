# Test if tf listner works correctly
using RobotOS
@rosimport geometry_msgs.msg: Wrench, Vector3
rostypegen(@__MODULE__)
import .geometry_msgs.msg: Wrench, Vector3

using ROSTransforms.tf
using Test

# Initialize node
init_node("listner", anonymous=true) # delete

# Declare an arbitrary transform
tf1 = Transform([1., 0, 0], [0, 0, 0, 1.])
wrench1 = Wrench(Vector3(0, 0, -1), Vector3(0, 1, 0))
wrench1_correct = Wrench(Vector3(0, 0, -1), Vector3(0, 2, 0))

# Transform that flips x/y to -x/-y
tf2 = Transform([1., 1, 0], [0, 0, 1, 0.])
wrench2 = Wrench(Vector3(0, 1, -1), Vector3(0, 0, 0))
wrench2_correct = Wrench(Vector3(0, -1, -1), Vector3(1, -1, 1))

@testset "Applying Transform" begin
    wrench_comp = transformWrench(tf1, wrench1)
    @test all([wrench_comp.force.x == wrench1_correct.force.x,
               wrench_comp.force.y == wrench1_correct.force.y,
               wrench_comp.force.z == wrench1_correct.force.z,
               wrench_comp.torque.x == wrench1_correct.torque.x,
               wrench_comp.torque.y == wrench1_correct.torque.y,
               wrench_comp.torque.z == wrench1_correct.torque.z])

    wrench_comp = transformWrench(tf2, wrench2)
    @test all([wrench_comp.force.x == wrench2_correct.force.x,
               wrench_comp.force.y == wrench2_correct.force.y,
               wrench_comp.force.z == wrench2_correct.force.z,
               wrench_comp.torque.x == wrench2_correct.torque.x,
               wrench_comp.torque.y == wrench2_correct.torque.y,
               wrench_comp.torque.z == wrench2_correct.torque.z])
end
