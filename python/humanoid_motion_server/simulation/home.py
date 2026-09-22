"""Adapt an optional simulation startup snapshot to the common pose schema."""

from ..poses import pose_from_joint_positions, resolve_initial_pose


HOME_POSE_ID = 'simulation_initial'


def resolve_home_pose(configuration):
    """Select a saved pose, or explicitly adapt the startup snapshot to a pose."""
    ident = configuration['home_pose_id']
    if not ident:
        return None
    document = configuration['pose_document']
    if ident == HOME_POSE_ID:
        if any(p['id'] == ident for p in document['initial_poses']):
            raise ValueError('simulation_initial is reserved for the simulation startup pose')
        pose = pose_from_joint_positions(
            document['resources'], configuration['initial'], pose_id=ident,
            name='仿真启动姿态', options=configuration['initial_pose_options'])
        document = {**document, 'initial_poses': [pose]}
    result = resolve_initial_pose(document, ident)
    for target in result['goals']:
        for name, value in zip(target['joint_names'], target['positions_rad']):
            joint = configuration['joints'][name]
            if not joint['lower'] <= value <= joint['upper']:
                raise ValueError(f'Home pose outside simulation limits: {name}')
    return result
