######################################################################
# gif.jl
#
# This file makes gifs of simulations
######################################################################

"""
`gif(m::SearchDomain, x::Vehicle, f::Filter, p::Policy, num_steps=10, filename=out.gif)`
"""
function gif(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int=10, filename="out.gif"; seconds_per_step=0.5, show_mean=false, show_cov=false, show_path=false)
	frames = Frames(MIME("image/png"), fps=20)
	path_x = Array(Float64,0)
	path_y = Array(Float64,0)

	# Plot the original scene
	plot(m, f, x)
	push!(frames, gcf())
	push!(path_x, x.x)
	push!(path_y, x.y)
	close()

	# 20 is the fps
	frames_per_step = round(Int, seconds_per_step * 20)

	for i = 1:num_steps
		old_pose = (x.x, x.y, x.heading)
		o = observe(m,x)
		update!(f, x, o)
		a = action(m, x, o, f, p)
		act!(m,x,a)
		new_pose = (x.x, x.y, x.heading)
		# Plot everything in between old pose and new pose
		dx = (new_pose[1] - old_pose[1]) / frames_per_step
		dy = (new_pose[2] - old_pose[2]) / frames_per_step
		dh = a[3] / frames_per_step
		for j = 1:frames_per_step
			#figure()
			# determine intermediate pose
			new_h = mod(old_pose[3] + dh, 360.0)
			old_pose = (old_pose[1] + dx, old_pose[2] + dy, new_h)

			push!(path_x, old_pose[1])
			push!(path_y, old_pose[2])
			plot(m, f, old_pose;show_mean=show_mean, show_cov=show_cov)
			if show_path
				plot(path_x, path_y)
			end
			push!(frames, gcf())
			close()
		end
	end
	write(filename, frames)
end

gif() = gif(Main.m, Main.x, Main.f, Main.p, 10)


function gif{TF<:AbstractFilter, TP<:Policy}(m::SearchDomain, xarr::Vector{Vehicle}, farr::Vector{TF}, parr::Vector{TP}, num_steps::Int=10, filename="out.gif"; seconds_per_step=0.5, show_mean=false, show_cov=false, show_path=false, alpha=1.0)
	num_vehicles = length(xarr)
	obsarr = zeros(num_vehicles)
	new_poses = Array(Pose, num_vehicles)
	old_poses = Array(Pose, num_vehicles)
	diffs = Array(Pose, num_vehicles)

	frames = Frames(MIME("image/png"), fps=20)

	# Set up the paths
	x_paths = Array(Vector{Float64}, num_vehicles)
	y_paths = Array(Vector{Float64}, num_vehicles)
	for (xi,x) in enumerate(xarr)
		x_paths[xi] = [x.x]
		y_paths[xi] = [x.y]
	end

	# Plot the original scene
	plot(m, farr, xarr)
	push!(frames, gcf())
	close()

	# 20 is the fps
	frames_per_step = round(Int, seconds_per_step * 20)

	colors = ["b", "r"]
	if num_vehicles == 1
		colors = ["b"]
	elseif num_vehicles == 2
		colors = ["b", "r"]
	elseif num_vehicles == 3
		colors = ["b", "r", "g"]
	elseif num_vehicles == 4
		colors = ["b", "r", "g", "k"]
	end

	for i = 1:num_steps
		for (xi,x) in enumerate(xarr)
			f = farr[xi]
			p = parr[xi]
			old_pose = (x.x, x.y, x.heading)
			old_poses[xi] = old_pose
			o = observe(m,x)
			update!(f, x, o)
			a = action(m, x, o, f, p)
			act!(m,x,a)
			new_pose = (x.x, x.y, x.heading)
			new_poses[xi] = new_pose
			# Plot everything in between old pose and new pose
			dx = (new_pose[1] - old_pose[1]) / frames_per_step
			dy = (new_pose[2] - old_pose[2]) / frames_per_step
			dh = a[3] / frames_per_step
			diffs[xi] = (dx,dy,dh)
		end
		for j = 1:frames_per_step
			#figure()
			# determine intermediate pose
			for (xi, x) in enumerate(xarr)
				dx = diffs[xi][1]
				dy = diffs[xi][2]
				dh = diffs[xi][3]
				old_pose = old_poses[xi]
				new_h = mod(old_pose[3] + dh, 360.0)
				old_pose = (old_pose[1] + dx, old_pose[2] + dy, new_h)
				old_poses[xi] = (old_pose)

				push!(x_paths[xi], old_pose[1])
				push!(y_paths[xi], old_pose[2])
			end
			plot(m, farr, old_poses; show_mean=show_mean, show_cov=show_cov, alpha=alpha)
			if show_path
				for (xi, x) in enumerate(xarr)
					plot(x_paths[xi], y_paths[xi], colors[xi])
				end
			end
			push!(frames, gcf())
			close()
		end
	end
	write(filename, frames)
end
