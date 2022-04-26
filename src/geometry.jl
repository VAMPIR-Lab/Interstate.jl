abstract type Shape end

struct Box2<:Shape
    x::Float64
    y::Float64
    θ::Float64
    lr::Float64
    lf::Float64
    w::Float64
end

function Box2(m::Movable)
    pos = position(m)
    Box2(pos[1], pos[2], heading(m), -rear(m), front(m), m.width)
end

function Box2(x_range, y_range)
    x = sum(x_range) / 2.0
    y = sum(y_range) / 2.0
    lr = x - x_range[1]
    lf = x_range[2] - x
    w = y_range[2]-y_range[1]
    Box2(x, y, 0, lr, lf, w)
end

function Box2(road::Road, segment_id; buf=10.0)
    segment = road.segments[segment_id]
    if isa(segment, CurvedSegment)
        error("Can't make a curved road segment into a box!")
    end
    start = segment.start
    finish = segment.finish
    vec = finish-start
    len = norm(vec)
    vec /= len
    start = start - buf*vec
    finish = finish + buf*vec
    perp = [vec[2], -vec[1]]
    w = road.lanes*road.lanewidth
    start_r = start + perp*w
    finish_r = finish + perp*w
    avg = 0.25 * (start+start_r+finish+finish_r)
    θ = atan(vec[2], vec[1])
    lr = lf = 0.5*(len+2*buf)
    Box2(avg..., θ, lr, lf, w)
end


struct Circle<:Shape
    x::Float64
    y::Float64
    r::Float64
end

struct Box3<:Shape
    x::Float64
    y::Float64
    z::Float64
    θ::Float64
    lr::Float64
    lf::Float64
    w::Float64
    h::Float64
end

function Box3(m::Movable)
    pos = position(m)
    Box3(pos[1], pos[2], 0.0, heading(m), -rear(m), front(m), m.width, m.height)
end

struct Ray2<:Shape
    # [x,y] + α [u,v]
    # α ≥ 0
    x::Float64
    y::Float64
    u::Float64
    v::Float64
end

struct Ray3<:Shape
    # [x,y,z] + α [u,v,w]
    # α ≥ 0
    x::Float64
    y::Float64
    z::Float64
    u::Float64
    v::Float64
    w::Float64
end

function Ray3(pos, beam)
    Ray3(pos..., beam...)
end

function intersect(ray::Ray3, box::Box3; max_dist = 100.0)
    intersect(box, ray, max_dist=max_dist)
end

function intersect(box::Box3, ray::Ray3; max_dist = 100.0)
    del = [box.x-ray.x, box.y-ray.y, box.z-ray.z]
    dist = norm(del)
    if dist > max_dist
        return (; collision=false, dist=0, p=[0,0,0], α=0)
    end

    I₃ = sparse(1.0I, 3, 3)
    P = [I₃ -I₃;
        -I₃ I₃]
    P = [P spzeros(6,1); spzeros(1,7)]

    c1 = cos(box.θ)
    s1 = sin(box.θ) 
    A1 = [c1 s1 0; -s1 c1 0; 0 0 1]
 
    b1 = A1*[box.x, box.y, box.z]
    l1 = [-box.lr, -box.w/2, 0] + b1
    u1 = [box.lf, box.w/2, box.h] + b1

    A = sparse([A1 spzeros(3,4);
        spzeros(3,3) sparse(I,3,3) -[ray.u, ray.v, ray.w];
        spzeros(1,6) 1.0])
    
    l = [l1; ray.x; ray.y; ray.z; 0.0]
    u = [u1; ray.x; ray.y; ray.z; Inf]
    
    mod = OSQP.Model()
    OSQP.setup!(mod; P=P, q=zeros(7), A=A, l=l, u=u, verbose=false, polish=true)
    results = OSQP.solve!(mod)

    dist=results.info.obj_val
    collision = dist < 1e-5
    p = results.x[1:3]
    α = results.x[7]
    
    (; collision, dist, p, α)
end

function intersection_over_union(m1, m2)
    c1 = get_corners(m1)
    h1 = convexhull(c1...)
    p1 = polyhedron(h1)

    c2 = get_corners(m2)
    h2 = convexhull(c2...)
    p2 = polyhedron(h2)
    Polyhedra.volume(Polyhedra.intersect(p1,p2)) / (Polyhedra.volume(p1) + Polyhedra.volume(p2))
end

function intersect(b1::Box2, b2::Box2)
    I₂ = sparse(1.0I, 2, 2)
    P = [I₂ -I₂;
        -I₂ I₂]
    c1 = cos(b1.θ)
    s1 = sin(b1.θ)
    c2 = cos(b2.θ)
    s2 = sin(b2.θ)
    A1 = [c1 s1; -s1 c1]
    A2 = [c2 s2; -s2 c2]
    A = [A1 spzeros(2,2); spzeros(2,2) A2]
    d = A*[b1.x, b1.y, b2.x, b2.y]
    l = [-b1.lr, -b1.w/2, -b2.lr, -b2.w/2] + d
    u = [b1.lf, b1.w/2, b2.lf, b2.w/2] + d

    mod = OSQP.Model()
    OSQP.setup!(mod; P=P, q=zeros(4), A=A, l=l, u=u, verbose=false, polish=true)
    results = OSQP.solve!(mod)

    dist=results.info.obj_val
    collision = dist < 1e-5
    p1 = results.x[1:2]
    p2 = results.x[3:4]

    (; collision, dist, p1, p2)
end

function intersect(circ::Circle, b::Box2)
    intersect(b, circ)
end

function intersect(b::Box2, circ::Circle)
    P = sparse(1.0I, 2, 2)
    q = -[circ.x, circ.y]
    c = cos(b.θ)
    s = sin(b.θ)
    A = sparse([c s; -s c])
    d = A*[b.x, b.y]
    l = [-b.lr, -b.w/2] + d
    u = [b.lf, b.w/2] + d

    mod = OSQP.Model()
    OSQP.setup!(mod; P=P, q=q, A=A, l=l, u=u, verbose=false, polish=true)
    results = OSQP.solve!(mod)

    p = results.x[1:2]
    dist = norm(p-[circ.x, circ.y])
    collision = dist < circ.r

    (; collision, dist, p)
end

function inside(pt::SVector{2,Float64}, b::Box2)
    c = cos(b.θ)
    s = sin(b.θ)
    A = [c s; -s c]
    d = A*[b.x, b.y]
    l = [-b.lr, -b.w/2] + d
    u = [b.lf, b.w/2] + d
    all(l .≤ A*pt .≤ u)
end

function inside(pt::SVector{2, Float64}, c::Circle)
    norm([pt[1]-c.x, pt[2]-c.y]) ≤ c.r
end

function box_corners(b::Box2)
    center = [b.x, b.y]
    c = cos(b.θ)
    s = sin(b.θ)
    A = [c -s; s c]
    p1 = SVector{2,Float64}(center + A * [-b.lr, -b.w/2])
    p2 = SVector{2,Float64}(center + A * [-b.lr, b.w/2])
    p3 = SVector{2,Float64}(center + A * [b.lf, -b.w/2])
    p4 = SVector{2,Float64}(center + A * [b.lf, b.w/2])
    pts = [p1, p2, p3, p4]
end
    

function inside(b1::Box2, b2::Box2)
    all(inside(pt, b2) for pt ∈ box_corners(b1))
end

function inside(box::Box2, circ::Circle)
    all(inside(pt, circ) for pt ∈ box_corners(box))
end
