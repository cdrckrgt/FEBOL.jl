using FEBOL
using Base.Test

# SearchDomain tests
m = SearchDomain(400, 170, 210)
@test m.theta == (170.0, 210.0)
m = SearchDomain(400)
theta!(m)
theta!(m, 32, 40)

# BearingOnly
s = BearingOnly()
@test s.noise_sigma == 10.0
s = BearingOnly(5)
@test s.noise_sigma == 5.0

# a trial simulation
m = SearchDomain(200)
s = FOV()
f = DF(m, 41, s, 0:1)
x = Vehicle(100, 100, 0, 5, s)
p = GreedyPolicy(x, 4)
simulate(m, x, f, p)

# simunit way
tc = StepThreshold(10)
su = SimUnit(x, f, p, tc)
