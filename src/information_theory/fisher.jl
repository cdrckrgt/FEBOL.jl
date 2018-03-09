
# fisher information
export fisher_det

# TODO: mega wrong, don't pass in the uav's sensor, pass in the filter's
fisher_det(uav::Vehicle, df::DF) = fisher_det(uav.sensor, df)

fisher_det(df) = fisher_det(df.sensor, df)

function fisher_det(bo::BearingOnly, df::DF)
	fd = zeros(df.n, df.n)
	ns2 = bo.noise_sigma * bo.noise_sigma

    # Loop over possible vehicle states
	for xi = 1:df.n, yi = 1:df.n
        x = (xi - 0.5) * df.cell_size
        y = (yi - 0.5) * df.cell_size

        # Loop over possible target states
        a = bc = d = 0.0
        for txi = 1:df.n, tyi = 1:df.n
            tx = (txi - 0.5) * df.cell_size
            ty = (tyi - 0.5) * df.cell_size
            p = df.b[txi,tyi]

            dx = tx - x
            dy = ty - y

            # ensure dx, dy are non-zero (to avoid singularities)
            dx == 0.0 && (dx = 1e-6)
            dy == 0.0 && (dy = 1e-6)

            r2 = dx*dx + dy*dy
            r4 = r2*r2
            
            a += p*dy*dy / r4
            bc -= p*dx*dy / r4
            d += p*dx*dx / r4
        end
        a /= ns2
        bc /= ns2
        d /= ns2

        fd[xi, yi] = a*d - bc*bc
	end

	return fd
end


function fisher_det(bo::RangeOnly, df::DF)
	fd = zeros(df.n, df.n)
	ns2 = bo.noise_sigma * bo.noise_sigma
	for xind = 1:df.n
		x = (xind - 0.5) * df.cell_size
		for yind = 1:df.n
			y = (yind - 0.5) * df.cell_size

			# now we must loop over belief
			a = bc = d = 0.0
			for txi = 1:df.n
				tx = (txi - 0.5) * df.cell_size
				for tyi = 1:df.n
					ty = (tyi - 0.5) * df.cell_size
					p = df.b[txi,tyi]
					dx = tx - x
					dy = ty - y
					if dx == 0.0
						dx = 1e-6
					end
					if dy == 0.0
						dy = 1e-6
					end
					r2 = dx*dx + dy*dy
					
					a += p*dx*dx / r2
					bc += p*dx*dy / r2
					d += p*dy*dy / r2
				end
			end
			a /= ns2
			bc /= ns2
			d /= ns2

			fd[xind, yind] = a*d - bc*bc

		end
	end
	return fd
end
