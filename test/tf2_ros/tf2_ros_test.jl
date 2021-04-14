#Test if tf listner works correctly
using RobotOS

# # Initialize node
# init_node("listner", anonymous=true) # delete

path = @__DIR__

# Need to run the listener and broadcaster in parralllel
script = (`julia --project=@. $path/tf2_ros_broadcaster.jl` &
          `julia --project=@. $path/tf2_ros_listener.jl`);
Base.run(script);
