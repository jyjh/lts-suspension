function ok = validateConfig(cfg)
% VALIDATECONFIG Validate the cfg.suspension struct consumed by this package.
%   ok = lts.components.Suspension.validateConfig(cfg)
%
% Contract item 2 of the repository split (see the Contracts page of the
% main repository's documentation): each component repository owns the
% schema of its cfg sub-struct and validates it at the build boundary.
%
% Top-level fields (SI, finite real scalars):
%   front/rear.springRate    [N/m]    wheel-domain heave spring (> 0)
%   front/rear.dampingCoeff  [N*s/m]  low-speed compression slope (>= 0)
%   front/rear.reboundCoeff  [N*s/m]  low-speed rebound slope (>= 0)
%   motionRatio              [-]      installation MR (0, 1]
%   bumpStopLength           [m]      free travel before the stop (>= 0)
%   bumpStopRate             [N/m]    bump-stop stiffness (>= 0)
%   tireSpringRate           [N/m]    vertical tire stiffness (> 0)
%   dampingKneeSpeed         [m/s]    optional; Inf = linear damper
%   dampingHighSpeedRatio    [-]      optional; high/low slope ratio
%   dampingReboundKneeSpeed  [m/s]    optional; NaN = none
%   frontArb/rearArb.stiffness [N*m/rad], .motionRatio (0,1], .leverArm
%                                     [m] (>= 0), .enabled (logical)
%   rollStiffnessOverride    NaN = derive, else fraction in [0,1]
%   coupleChassisRollToLoadTransfer (logical)
%   geometry                 struct (front/rear/steering kinematics; see
%                            the Contracts page for the full field list)
%
% Returns logical true on success; otherwise throws with identifier
% lts_suspension_validateConfig:<Case> (MissingField | InvalidScalar |
% OutOfRange | InvalidGeometry).

required = {'motionRatio', 'bumpStopLength', 'bumpStopRate', ...
    'tireSpringRate', 'frontArb', 'rearArb', ...
    'rollStiffnessOverride', 'coupleChassisRollToLoadTransfer'};
for i = 1:numel(required)
    if ~isfield(cfg, required{i}) || isempty(cfg.(required{i}))
        error('lts_suspension_validateConfig:MissingField', ...
            'cfg.suspension.%s is required.', required{i});
    end
end

axles = {'front', 'rear'};
for a = 1:2
    axle = axles{a};
    if ~isfield(cfg, axle)
        error('lts_suspension_validateConfig:MissingField', ...
            'cfg.suspension.%s is required.', axle);
    end
    parts = {'springRate', 'dampingCoeff', 'reboundCoeff'};
    for i = 1:numel(parts)
        name = sprintf('%s.%s', axle, parts{i});
        if ~isfield(cfg.(axle), parts{i})
            error('lts_suspension_validateConfig:MissingField', ...
                'cfg.suspension.%s is required.', name);
        end
        localCheckScalar(cfg.(axle), parts{i}, name);
    end
    if cfg.(axle).springRate <= 0
        error('lts_suspension_validateConfig:OutOfRange', ...
            'cfg.suspension.%s.springRate must be > 0.', axle);
    end
    if cfg.(axle).dampingCoeff < 0 || cfg.(axle).reboundCoeff < 0
        error('lts_suspension_validateConfig:OutOfRange', ...
            'cfg.suspension.%s damping coefficients must be >= 0.', axle);
    end
end

localCheckScalar(cfg, 'motionRatio');
localCheckScalar(cfg, 'bumpStopLength');
localCheckScalar(cfg, 'bumpStopRate');
localCheckScalar(cfg, 'tireSpringRate');
if cfg.motionRatio <= 0 || cfg.motionRatio > 1
    error('lts_suspension_validateConfig:OutOfRange', ...
        'cfg.suspension.motionRatio=%g must be in (0, 1].', cfg.motionRatio);
end
if cfg.bumpStopLength < 0 || cfg.bumpStopRate < 0 || cfg.tireSpringRate <= 0
    error('lts_suspension_validateConfig:OutOfRange', ...
        ['bumpStopLength/bumpStopRate must be >= 0 and tireSpringRate ' ...
        'must be > 0.']);
end

optional = {'dampingKneeSpeed', 'dampingHighSpeedRatio', ...
    'dampingReboundKneeSpeed'};
for i = 1:numel(optional)
    if isfield(cfg, optional{i}) && ~isempty(cfg.(optional{i})) ...
            && ~isnan(cfg.(optional{i}))
        localCheckScalar(cfg, optional{i});
    end
end

bars = {'frontArb', 'rearArb'};
for b = 1:2
    bar = cfg.(bars{b});
    parts = {'stiffness', 'motionRatio', 'leverArm', 'enabled'};
    for i = 1:numel(parts)
        if ~isfield(bar, parts{i})
            error('lts_suspension_validateConfig:MissingField', ...
                'cfg.suspension.%s.%s is required.', bars{b}, parts{i});
        end
    end
    if ~isscalar(bar.stiffness) || ~isfinite(bar.stiffness) || ...
            bar.stiffness < 0
        error('lts_suspension_validateConfig:InvalidScalar', ...
            'cfg.suspension.%s.stiffness must be finite and >= 0.', bars{b});
    end
    if ~isscalar(bar.motionRatio) || ~isfinite(bar.motionRatio) || ...
            bar.motionRatio <= 0 || bar.motionRatio > 1
        error('lts_suspension_validateConfig:OutOfRange', ...
            'cfg.suspension.%s.motionRatio must be in (0, 1].', bars{b});
    end
    if ~isscalar(bar.leverArm) || ~isfinite(bar.leverArm) || ...
            bar.leverArm < 0
        error('lts_suspension_validateConfig:InvalidScalar', ...
            'cfg.suspension.%s.leverArm must be finite and >= 0.', bars{b});
    end
    if ~islogical(bar.enabled) && ~isnumeric(bar.enabled)
        error('lts_suspension_validateConfig:InvalidScalar', ...
            'cfg.suspension.%s.enabled must be logical.', bars{b});
    end
end

override = cfg.rollStiffnessOverride;
if ~(isnan(override) && isscalar(override)) && ...
        ~(isscalar(override) && isfinite(override) && ...
          override >= 0 && override <= 1)
    error('lts_suspension_validateConfig:OutOfRange', ...
        'rollStiffnessOverride must be NaN (derive) or in [0, 1].');
end
if ~islogical(cfg.coupleChassisRollToLoadTransfer)
    error('lts_suspension_validateConfig:InvalidScalar', ...
        'coupleChassisRollToLoadTransfer must be logical.');
end

if ~isfield(cfg, 'geometry') || ~isstruct(cfg.geometry)
    error('lts_suspension_validateConfig:InvalidGeometry', ...
        'cfg.suspension.geometry (front/rear/steering) is required.');
end
localCheckGeometry(cfg.geometry);

ok = true;
end

function localCheckScalar(owner, field, label)
value = owner.(field);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error('lts_suspension_validateConfig:InvalidScalar', ...
        'cfg.suspension.%s must be a finite real scalar (got %s).', ...
        label, mat2str(value));
end
end

function localCheckGeometry(geom)
% Presence + basic shape of the kinematics tables. Deep validation
% (monotonic grids, curve lengths) belongs to SuspensionGeometry.
required = {'front', 'rear', 'steering'};
for i = 1:numel(required)
    if ~isfield(geom, required{i}) || ~isstruct(geom.(required{i}))
        error('lts_suspension_validateConfig:InvalidGeometry', ...
            'cfg.suspension.geometry.%s is required.', required{i});
    end
end
axleFields = {'travelGrid', 'camberCurve', 'toeCurve', ...
    'motionRatioCurve', 'rollCenterHeight'};
axles = {'front', 'rear'};
for a = 1:2
    for i = 1:numel(axleFields)
        f = axleFields{i};
        if ~isfield(geom.(axles{a}), f)
            error('lts_suspension_validateConfig:InvalidGeometry', ...
                'cfg.suspension.geometry.%s.%s is required.', axles{a}, f);
        end
    end
end
steeringFields = {'steeringRatio', 'ackermann', 'maxWheelSteerAngle'};
for i = 1:numel(steeringFields)
    f = steeringFields{i};
    if ~isfield(geom.steering, f)
        error('lts_suspension_validateConfig:InvalidGeometry', ...
            'cfg.suspension.geometry.steering.%s is required.', f);
    end
end
end
