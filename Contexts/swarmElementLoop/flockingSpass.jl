using StaticArrays, LinearAlgebra, Statistics, Plots

# Roboterdefinition
struct Robot
    id::Int
    pos::SVector{2,Float64}
    vel::SVector{2,Float64}
end

# Separation: vermeide Nähe
function separation(r::Robot, neighbors; r_sep=1.5)
    sep = SVector(0.0, 0.0)
    for n in neighbors
        diff = r.pos - n.pos
        dist = norm(diff)
        if dist < r_sep && dist > 0
            sep += diff / dist^2
        end
    end
    return sep
end

# Alignment: gleiche Bewegungsrichtung an
function alignment(r::Robot, neighbors)
    return SVector(0.0, 0.0)
end

# Cohesion: bewege dich zum Zentrum
function cohesion(r::Robot, neighbors)
    isempty(neighbors) && return SVector(0.0, 0.0)
    center = mean([n.pos for n in neighbors])
    return center - r.pos
end

# Zielberechnung: Position anstatt Geschwindigkeit
function compute_target(r::Robot, neighbors;
                        w_sep=1.5, w_ali=1.0, w_coh=1.0, step_size=0.5)

    dir = w_sep * separation(r, neighbors) +
          w_ali * alignment(r, neighbors) +
          w_coh * cohesion(r, neighbors)

    if norm(dir) == 0
        return r.pos
    end

    dir = normalize(dir)
    return r.pos + step_size * dir
end

# Bewegung zum Ziel
function move_toward(r::Robot, target; speed=0.1)
    dir = target - r.pos
    dist = norm(dir)
    if dist < speed || dist == 0
        new_pos = target
    else
        new_pos = r.pos + normalize(dir) * speed
    end
    return Robot(r.id, new_pos, normalize(dir))
end

# Gesamtschritt
function flock_step(robots; neighbor_range=5.0)
    targets = Dict{Int,SVector{2,Float64}}()
    for r in robots
        neighbors = [n for n in robots if n.id != r.id && norm(r.pos - n.pos) < neighbor_range]
        targets[r.id] = compute_target(r, neighbors)
    end
    return [move_toward(r, targets[r.id]) for r in robots]
end

# Simulation starten
function simulate_flock(n=10, steps=100)
    robots = [Robot(i, @SVector[rand()*10, rand()*10], @SVector[0.0, 0.0]) for i in 1:n]
    anim = @animate for t in 1:steps
        robots = flock_step(robots)
        xs = [r.pos[1] for r in robots]
        ys = [r.pos[2] for r in robots]
        scatter(xs, ys, xlim=(0, 10), ylim=(0, 10),
                title="Flocking step $t", legend=false, aspect_ratio=:equal)
    end
    return anim
end

# Animation erzeugen
anim = simulate_flock(10, 150)
gif(anim, "flocking_positions.gif", fps=20)