######################################################################
# gif.jl
#
# This file makes gifs of simulations
######################################################################

"""
`gif(m::SearchDomain, x::Vehicle, f::Filter, p::Policy, num_steps=10, filename=out.gif)`
"""
function gif(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int=10, filename="out.gif"; seconds_per_step=0.5)
	frames = Frames(MIME("image/png"), fps=20)

	# Plot the original scene
	plot(m, f, x)
	push!(frames, gcf())
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
		dx = (new_pose[1] - old_pose[1]) / 10.0
		dy = (new_pose[2] - old_pose[2]) / 10.0
		dh = a[3] / 10.0
		for j = 1:frames_per_step
			#figure()
			# determine intermediate pose
			new_h = mod(old_pose[3] + dh, 360.0)
			old_pose = (old_pose[1] + dx, old_pose[2] + dy, new_h)

			plot(m, f, old_pose)
			push!(frames, gcf())
			close()
		end
	end
	write(filename, frames)
end

gif() = gif(Main.m, Main.x, Main.f, Main.p, 10)
