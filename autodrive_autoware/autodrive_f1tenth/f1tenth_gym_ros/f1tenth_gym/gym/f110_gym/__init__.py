from gym.envs.registration import register
register(
	id='f110-v0',
	entry_point='f110_gym.envs:F110Env',
	)