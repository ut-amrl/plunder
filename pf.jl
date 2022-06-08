using CUDA: @allowscalar

effective_particles(weights::AbstractVector) = 1 / sum(abs2, weights)

function systematic_resample!(indices, weights::AbsVec, bins_buffer::AbsVec)
    N = length(weights)
    M = length(indices)
    @smart_assert length(bins_buffer) == N
    cumsum!(bins_buffer, weights)
    delta = @allowscalar bins_buffer[end] / M
    r = delta * rand()
    broadcast!(indices, 0:(M - 1)) do i
        s = delta * i + r
        min(searchsortedfirst(bins_buffer, s), N)
    end
    return indices
end

"""
Sampling n indices according to their weights. 
See https://youtu.be/tvNPidFMY20 for an explanation of the algorithm.
"""
function systematic_resample(weights::AbstractArray{N}, n_samples::Integer) where {N}
    if !isfinite(weights)
        @error "Weights must be finite." weights
        throw(ArgumentError("Weights must be finite."))
    end
    indices = similar(weights, Int, n_samples)
    indices .= 1:n_samples
    bins_buffer = similar(weights)
    systematic_resample!(indices, weights, bins_buffer)
end

"""
Run particle filters forward in time
"""
function forward_filter(
    system::MarkovSystem{X},
    (; times, obs_frames, controls, observations),
    n_particles;
    resample_threshold::Float64=0.5,
    score_f=logpdf,
    showprogress=true,
    cache::Dict=Dict(),
    check_finite::Bool=false,
) where {X}
    @smart_assert eltype(obs_frames) <: Integer
    T, N = length(times), n_particles
    (; x0_dist, motion_model, obs_model) = system

    particles::Matrix{X} = get!(cache, :particles) do
        Matrix{X}(undef, N, T)
    end
    ancestors::Matrix{Int} = get!(cache, :ancestors) do
        Matrix{Int}(undef, N, T)
    end

    particles[:, 1] .= (rand(x0_dist) for _ in 1:N)
    ancestors[:, :] .= 1:N

    log_weights = fill(-log(N), N) # log weights at the current time step
    weights = exp.(log_weights)
    log_obs::Float64 = 0.0  # estimation of the log observation likelihood
    bins_buffer = zeros(Float64, N) # used for systematic resampling

    progress = Progress(T; desc="forward_filter", output=stdout, enabled=showprogress)
    for t in 1:T
        if t in obs_frames
            for i in 1:N
                x_i = particles[i, t]
                obs_dist = obs_model(x_i)
                log_obs_ti = score_f(obs_dist, observations[t])
                if check_finite && !isfinite(log_obs_ti)
                    @error "log_obs_ti is not finite" x_i obs = observations[t] obs_dist
                    throw(ErrorException("log_obs_ti is not finite."))
                end
                log_weights[i] += log_obs_ti
            end
            log_z_t = logsumexp(log_weights)
            log_weights .-= log_z_t
            weights .= exp.(log_weights)
            log_obs += log_z_t

            if check_finite && !(abs(sum(weights) - 1.0) < 1e-6)
                @error "Weights must sum to one." t sum = sum(weights) log_z_t
                @error "Weights must sum to one." weights log_weights
                @error "Weights must sum to one." particles = particles[:, t]
                throw(ErrorException("Bad weights produced."))
            end

            # optionally resample
            if effective_particles(weights) < N * resample_threshold
                indices = @views(ancestors[:, t])
                systematic_resample!(indices, weights, bins_buffer)
                log_weights .= -log(N)
                particles[:, t] = particles[indices, t]
            end
        end

        if t < T
            Δt = times[t + 1] - times[t]
            u = controls[t]

            for i in 1:N
                particles[i, t + 1] = rand(motion_model(particles[i, t], u, Δt))
            end
        end
        next!(progress)
    end

    log_obs::Float64
    (; particles, weights, log_weights, ancestors, log_obs)
end


"""
Sample smooting trajecotries by tracking the ancestors of a particle filter.
"""
function filter_smoother(
    system,
    obs_data;
    n_particles,
    n_trajs,
    resample_threshold=0.5,
    score_f=logpdf,
    use_auxiliary_proposal::Bool=false,
    showprogress=true,
    cache::Dict=Dict(),
)
    (; particles, weights, ancestors, log_obs) = forward_filter(
        system,
        obs_data,
        n_particles;
        resample_threshold,
        score_f,
        use_auxiliary_proposal,
        cache,
        showprogress,
    )
    traj_ids = systematic_resample(weights, n_trajs)
    (; trajectories, n_effective) = particle_trajectories(particles, ancestors, traj_ids)
    (; trajectories, n_effective, log_obs)
end

"""
From the give particle `indices` at the last time step, going backward to trace
out the ancestral lineages. 
"""
function particle_trajectories(
    particles::Matrix{P}, ancestors::Matrix{Int}, indices::Vector{Int}
) where {P}
    N, T = size(particles)
    indices = copy(indices)
    n_unique = 0
    n_unique += length(Set(indices))
    M = length(indices)
    trajs = Matrix{P}(undef, M, T)
    trajs[:, T] = particles[indices, T]
    for t in (T - 1):-1:1
        indices .= ancestors[indices, t + 1]
        n_unique += length(Set(indices))
        trajs[:, t] = particles[indices, t]
    end
    trajectories::Vector{Vector{P}} = get_rows(trajs) |> collect
    n_effective = n_unique / T
    (; trajectories, n_effective)
end

"""
Particle smoother based on the Forward filtering-backward sampling algorithm.
"""
function ffbs_smoother(
    system::MarkovSystem{X},
    obs_data;
    n_particles,
    n_trajs,
    resample_threshold::Float64=0.5,
    score_f=logpdf,
    progress_offset=0,
) where {X}
    (; times, obs_frames, controls, observations) = obs_data

    function forward_logp(x_t, x_t′, t)
        local Δt = times[t + 1] - times[t]
        local d = system.motion_model(x_t, controls[t], Δt)
        score_f(d, x_t′)
    end

    (; particles, log_weights, log_obs) = forward_filter(
        system, obs_data, n_particles; resample_threshold, score_f
    )
    log_obs::Float64
    trajectories = backward_sample(
        forward_logp, particles, log_weights, n_trajs; progress_offset
    )
    (; trajectories, log_obs)
end


"""
Performs the backward recursion of the Forward filtering-backward sampling algorithm
to sample from the smoothing distribution.
## Arguments
- `forward_logp(x_t, x_t′, t)` should return the log probability of the transition dynamics.
"""
function backward_sample(
    forward_logp::Function,
    filter_particles::Matrix{P},
    filter_log_weights::Matrix{<:Real},
    n_trajs::Integer;
    progress_offset=0,
) where {P}
    M = n_trajs
    N, T = size(filter_particles)
    trajectories = Matrix{P}(undef, M, T)
    weights = [softmax(filter_log_weights[:, T]) for _ in 1:M]

    trajectories[:, T] .= (
        let j = rand(Categorical(weights[j]))
            filter_particles[j, T]
        end for j in 1:M
    )

    progress = Progress(
        T - 1; desc="backward_sample", offset=progress_offset, output=stdout
    )
    for t in (T - 1):-1:1
        Threads.@threads for j in 1:M
            weights[j] .=
                @views(filter_log_weights[:, t]) .+
                forward_logp.(
                    @views(filter_particles[:, t]), Ref(trajectories[j, t + 1]), t
                )
            softmax!(weights[j])
            j′ = rand(Categorical(weights[j]))
            trajectories[j, t] = filter_particles[j′, t]
        end
        next!(progress)
    end
    trajectories
end