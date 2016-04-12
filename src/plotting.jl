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
	plot_vehicle(x)
	plot(m, f)
	return # so it doesn't spit out result of axis
end

function plot(m::SearchDomain, f::DF)
	a = [0,m.length,0,m.length]
	imshow(f.b', interpolation="none",cmap="Greys",origin="lower",extent=a)
	labels()
	axis(a)
end

function plot(m::SearchDomain, f::EKF)
	a = [0,m.length,0,m.length]

	# Plot it out...
	xvals = linspace(0.,m.length,100)
	yvals = linspace(0.,m.length,100)
	n = length(xvals)
	gaussian_arr = zeros(n, n)
	#d = MvNormal(f.mu, sqrt(f.Sigma))
	d = MvNormal(f.mu, f.Sigma)
	for x = 1:n
		for y = 1:n
			gaussian_arr[x,y] = pdf(d, [xvals[x],yvals[y]])
		end
	end
	plot_contour(m, gaussian_arr)

	labels()
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
	scatter(x,y,c=f.W,cmap="Greys")
	labels()
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
function plot_vehicle(x::Vehicle)
	mark_size = 12
	plot(x.x, x.y, "b*", markersize=mark_size)
end

# Plots jammer location
function plot_theta(m::SearchDomain)
	mark_size = 12
	plot(m.theta[1], m.theta[2], "r^", markersize=mark_size)
end

# Plots contours of some distribution `d` (a matrix).
function plot_contour(m::SearchDomain, d::Matrix{Float64})
	X,Y = meshgrid(linspace(0.,m.length,100), linspace(0.,m.length,100))
	contour(X, Y, d')
end

# Sets the appropriate plot labels
function labels()
	xlabel("x")
	ylabel("y")
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
