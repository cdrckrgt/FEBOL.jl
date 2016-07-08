######################################################################
# gps.jl
# handles some gps functions
######################################################################

export gps_dist
export gps_bearing
export gps_offset
export gps_sim
export parse_log

"""
`gps_dist(latlon1::LocTuple, latlon2::LocTuple)`

Returns a scalar distance between these two points (I think it won't be -ve)
"""
function gps_dist(latlon1::LocTuple, latlon2::LocTuple)
	lat1 = latlon1[1]
	lon1 = latlon1[2]
	lat2 = latlon2[1]
	lon2 = latlon2[2]

	a = sind( (lat2-lat1)/2 )^2 + cosd(lat1)*cosd(lat2) * sind((lon2-lon1)/2)^2
	c = 2atan2(sqrt(a), sqrt(1-a))
	R = 6371000   # earth distance in meters
	d = R*c
	return d
end

"""
`gps_bearing(latlon1::LocTuple, latlon2::LocTuple)`

where `LocTuple` is a tuple of two floats.
Returns bearing from latlon1 to latlon2, east of north.
"""
function gps_bearing(latlon1::LocTuple, latlon2::LocTuple)
	lat1 = latlon1[1]
	lon1 = latlon1[2]
	lat2 = latlon2[1]
	lon2 = latlon2[2]


	y = sind(lon2-lon1)*cosd(lat2)
	x = cosd(lat1)*sind(lat2) - sind(lat1)*cosd(lat2)*cosd(lon2-lon1)
	return mod(rad2deg(atan2(y,x)), 360.0)
end

"""
`gps_offset(latlon1::LocTuple, latlon2::LocTuple)`

Returns the dx, dy offset from `latlon1` to `latlon2`.
"""
function gps_offset(latlon1::LocTuple, latlon2::LocTuple)
	d = gps_dist(latlon1, latlon2)
	b = gps_bearing(latlon1, latlon2)
	dx = d * sind(b)
	dy = d * cosd(b)
	return dx,dy
end

# create simulation and drawing from
# j is the jammer start point (lat, lon) decimal gps coords
# x0 is the uav start point (lat, lon) decimal gps coords
function gps_sim(j::LocTuple, x0::LocTuple, actions::Vector{Pose}, observations::Vector{Float64}, m::SearchDomain, x::Vehicle, f::AbstractFilter; show_mean::Bool=false, show_cov::Bool=false)

	# always start the vehicle in the center
	x.x = m.length / 2.0
	x.y = m.length / 2.0
	dx,dy = gps_offset(x0, j)
	theta!(m, (x.x+dx, x.y+dy))

	# warn user if jammer is not where it should be
	if abs(dx) > x.x || abs(dy) > x.y
		println("WARNING: jammer outside search domain.")
	end

	# loop through all observations...
	for (oi,o) in enumerate(observations)
		update!(f, x, o)
		plot(m, f, x,show_mean=show_mean,show_cov=show_cov)
		savefig("temp_$(oi).png", format="png")
		hold(false)
		act!(m, x, actions[oi])
	end
end

function gps_sim(j::LocTuple, x0::LocTuple, actions::Vector{Pose}, observations::Vector{Float64}, b::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle)
	# always start the vehicle in the center
	x.x = m.length / 2.0
	x.y = m.length / 2.0
	dx,dy = gps_offset(x0, j)
	theta!(m, (x.x+dx, x.y+dy))

	# warn user if jammer is not where it should be
	if abs(dx) > x.x || abs(dy) > x.y
		println("WARNING: jammer outside search domain.")
	end

	# loop through all observations...
	for (oi,o) in enumerate(observations)
		#update!(f, x, o)
		#plot(m, b[oi], x, show_mean=true, show_cov=true)
		plot(m, b[oi], x, show_mean=false, show_cov=false)
		savefig("temp_$(oi).png", format="png")
		hold(false)
		act!(m, x, actions[oi])
	end
end

# This is not really gps related
# Reads my log file
# TODO: handle different types of beliefs
function parse_log(filename::ASCIIString)

	obs = Array(Float64, 0)
	actions = Array(Pose, 0)
	beliefs = Array(Matrix{Float64}, 0)

	logfile = open(filename, "r")

	# create the filter and stuff
	line = readline(logfile)
	while !contains(line, "#####")
		# read header stuff

		line = readline(logfile)
	end

	# now get the actions, observations and beliefs
	while (line = readline(logfile)) != ""
		if contains(line, "observation")
			# observation line
			push!(obs, parse(Float64, split(line, " = ")[2]) )
		elseif contains(line, "action")
			# action line
			a_vec = split(split(line, " = ")[2], ",")
			# the actions we get are north,east,yaw. we want dx,dy,yaw
			dx  = parse(Float64, a_vec[2])
			dy  = parse(Float64, a_vec[1])
			dh  = parse(Float64, a_vec[3])
			push!(actions, (dx, dy, dh))
		elseif contains(line, "belief")
			# now read the next bunch of lines
			bline = readline(logfile)
			arr = split(bline, ",")
			n = length(arr)
			b = Array(Float64, n, n)
			for i = 1:n
				b[i,n] = parse(Float64, arr[i])
			end
			for i = 2:n
				bline = readline(logfile)
				arr = split(bline, ",")
				for j = 1:n
					b[j,n-i+1] = parse(Float64, arr[j])
				end
			end
			push!(beliefs, b)
		end
	end
	close(logfile)

	return obs,actions,beliefs
end
