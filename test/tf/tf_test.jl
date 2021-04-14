#Test if tf listner works correctly
using RobotOS
#
# # Initialize node
# init_node("listner", anonymous=true) # delete

path = @__DIR__

# Need to run the listener and broadcaster in parralllel
script = (`julia --project=@. $path/tf_listener.jl` &
          `julia --project=@. $path/tf_broadcaster.jl` &
          `julia --project=@. $path/tf_applied.jl`);
Base.run(script);
