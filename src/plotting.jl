######################################################################
# plotting.jl
# Handles all the calls to PyPlot to generate plots
######################################################################

"""
`plot(m::SearchDomain, f::AbstractFilter, x::Vehicle)`

Plots the belief, jammer, and vehicles.
"""
function plot(m::SearchDomain, f::AbstractFilter, x::Vehicle)
	plot_theta(m)
	hold(true)
	plot_vehicle(m,x)
	plot(m, f)
	return # so it doesn't spit out result of axis
end
function plot(m::SearchDomain, f::AbstractFilter, p::Pose)
	plot_theta(m)
	hold(true)
	plot_vehicle(m,p)
	plot(m, f)
	return # so it doesn't spit out result of axis
end

function plot(m::SearchDomain, f::DF)
	a = [0,m.length,0,m.length]
	imshow(f.b', interpolation="none",cmap="Greys",origin="lower",extent=a,vmin=0)
	labels()
	axis(a)
end

# Plots mean and 95% confidence ellipse
function plot(m::SearchDomain, f::EKF)
	a = [0,m.length,0,m.length]

	Sigma_half = sqrtm(f.Sigma)
	Sh11 = Sigma_half[1,1]
	Sh12 = Sigma_half[1,2]
	Sh21 = Sigma_half[2,1]
	Sh22 = Sigma_half[2,2]
	m1 = f.mu[1]
	m2 = f.mu[2]

	thetas = 0:.02:2pi
	num_theta = length(thetas)
	xvals = zeros(num_theta)
	yvals = zeros(num_theta)
	c = sqrt(-2.0*log(0.05))
	for i = 1:num_theta
		theta = thetas[i]
		s_theta = sin(theta)
		c_theta = cos(theta) 
		xvals[i] = c*(Sh11*s_theta + Sh12*c_theta) + m1
		yvals[i] = c*(Sh21*s_theta + Sh22*c_theta) + m2
	end
	plot(xvals, yvals, "g")
	plot(m1, m2, "gx", ms=10)

	labels()
	axis("square")
	axis(a)
end

function plot(m::SearchDomain, f::PF)
	mark_size = 12
	a = [0,m.length,0,m.length]
	x = zeros(f.n)
	y = zeros(f.n)
	for i = 1:f.n
		x[i] = f.X[i][1]
		y[i] = f.X[i][2]
	end
	scatter(x,y,c=f.W,cmap="Greys",vmin=0)
	#scatter(x,y,c=f.W,cmap="Blues",vmin=0)
	xmean, ymean = centroid(f)
	plot(xmean, ymean, "gx", ms=10, mew=2)
	labels()
	axis("square")
	axis(a)
end



# This is needed for plot_sim
#function plot_b(m::SearchDomain, b::Belief, x::Vehicle)
#	plot_theta(m)
#	hold(true)
#	plot_vehicles(X)
#	imshow(b', interpolation="none", cmap="Greys", origin="lower")
#	labels()
#end

#"""
#`plot_eid(m::SearchDomain, X::VehicleSet)`
#
#Plots the eid using the Fisher information matrix.
#"""
#function plot_eid(m::SearchDomain, X::VehicleSet; contours=false)
#	eid = EID(m,m.b)
#	if contours
#		plot_contour(m, eid)
#	end
#	plot_theta(m)
#	plot_vehicles(X)
#	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
#	labels()
#end

#"""
#`plot_mi(m::SearchDomain, X::VehicleSet)`
#
#Plots the mutual information.
#"""
#function plot_mi(m::SearchDomain, X::VehicleSet; contours=false)
#	mut_info = mutual_information(m)
#	plot_vehicles(X)
#	imshow(mut_info', interpolation="none", cmap="Greys", origin="lower")
#	labels()
#	if contours
#		plot_contour(m, mut_info)
#	end
#end


#"""
#`plot_sim(m::SearchDomain, s::Simulation)`
#
#Steps through a simulation.
#"""
#function plot_sim(m::SearchDomain, s::Simulation)
#	for t = 0:s.T
#		hold(false)
#		pause(1)
#		plot_b(m, s.belief_list[t+1], s.state_list[t+1])
#	end
#end


######################################################################
# Helper functions
######################################################################
# Plots locations of the vehicles
plot_vehicle(m::SearchDomain,x::Vehicle) = plot_vehicle(m,x.x,x.y,x.heading)
plot_vehicle(m::SearchDomain, p::Pose) = plot_vehicle(m, p[1], p[2], p[3])
function plot_vehicle(m::SearchDomain, x::Float64, y::Float64, h::Float64)
	mark_size = 10
	#plot(x.x, x.y, "bx", markersize=mark_size, mew=2)

	c = 0.01 * m.length  *sqrt(2)
	dx = c*sind(45 + h)
	dy = c*cosd(45 + h)

	# Plot rotors
	rotor_size = 5
	theta1 = 45.0 + h
	theta2 = 135.0 + h
	theta3 = 225.0 + h
	theta4 = 315.0 + h
	plot(x+c*sind(theta1), y+c*cosd(theta1), "bo", ms=rotor_size)
	plot(x+c*sind(theta2), y+c*cosd(theta2), "bo", ms=rotor_size)
	plot(x+c*sind(theta3), y+c*cosd(theta3), "bo", ms=rotor_size)
	plot(x+c*sind(theta4), y+c*cosd(theta4), "bo", ms=rotor_size)

	#plot(x.x, x.y, marker=(2,0,3), ms=mark_size)

	# plot direction
	xline = [x, x+2*c*sind(h)]
	yline = [y, y+2*c*cosd(h)]
	#plot(xline, yline, "r", mew=1.5)
	plot(xline, yline, "r", mew=2)

	# Plot frame
	plot(x, y, marker=(2,0,-theta1), ms=mark_size,mew=1,markeredgecolor="b")
	plot(x, y, marker=(2,0,-theta2), ms=mark_size,mew=1,markeredgecolor="b")

	#plot()
end

# Plots jammer location
function plot_theta(m::SearchDomain)
	mark_size = 11
	plot(m.theta[1], m.theta[2], "r^", markersize=mark_size)
	#mark_size = 11
	#plot(m.theta[1], m.theta[2], "r^", markersize=mark_size, markerfacecolor="none", markeredgecolor="r", mew=2)
end

# Plots contours of some distribution `d` (a matrix).
function plot_contour(m::SearchDomain, d::Matrix{Float64})
	X,Y = meshgrid(linspace(0.,m.length,100), linspace(0.,m.length,100))
	contour(X, Y, d')
end

# Sets the appropriate plot labels
function labels()
	xlabel("x (meters)")
	ylabel("y (meters)")
end

######################################################################
# Old, deprecated, or unused
######################################################################
function plot_eid2(m, b, x, theta)
	mark_size = 12
	eid = EID(m,theta[1],theta[2])
	plot(x[1], x[2], "b*", markersize=mark_size)
	plot(theta[1], theta[2], "r^", markersize=mark_size)
	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
	xlabel("x")
	ylabel("y")
end

function plot_eid3(m, b, x, theta)
	mark_size = 12
	eid = EID(m,theta[1],theta[2])
	#plot(x[1], m.num_cells - 1 - x[2], "b*", markersize=mark_size)
	#plot(theta[1], m.num_cells - 1 - theta[2], "r^", markersize=mark_size)
	#eid_plot = (eid')[end:-1:1, :]

	#imshow(eid_plot, interpolation="none", cmap="Greys")
	plot_contour(m, eid)
	xlabel("x")
	ylabel("y")
end

"""
`meshgrid(x, y)`

Returns X and Y.
"""
function meshgrid(x, y)
	nx = length(x)
	ny = length(y)

	X = zeros(ny, nx)
	Y = zeros(ny, nx)
	for xi = 1:nx
		for xj = 1:ny
			X[xj, xi] = x[xi]
			Y[xj, xi] = y[xj]
		end
	end
	return X, Y
end
