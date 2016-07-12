from collections import namedtuple
from dxlchain import DxlChain

TargetInfo = namedtuple("TargetInfo", ["pos", "time"])
MotorTarget = namedtuple("MotorTarget", ["motor_id", "pos"])
TimeTarget = namedtuple("TimeTarget", ["time", "motor_targets"])


class TimeTarget:
	def __init__(self):
		self.ids = []  # type: list[int]
		self.times = [] # type: list[float]
		self.trajs = [] # type: list[list[float]]


class TrajectoryControl:
	from timeit import default_timer

	def __init__(self, chain):
		self._chain = chain  # type: DxlChain
		self._fast_move_time_threshold = 0.5

	def multi_traj_move(self, time_target, begin_time=None):
		"""
		:type time_target: TimeTarget
		:type begin_time: float
		"""
		if not time_target.times:
			return

		# TODO: wait to begin_time

		# init fast speeds
		speeds_max = [0] * len(time_target.ids)
		in_max_spd = False

		beg_time = self.default_timer()
		for time_tar, positions in zip(time_target.times, time_target.trajs):
			time_now = self.default_timer() - beg_time
			time_diff = time_tar - time_now

			# Move
			if time_diff > self._fast_move_time_threshold:
				# move with speed control
				for m_id, pos in zip(time_target.ids, positions):
					self._move_to( m_id, pos, time_tar, time_now, move_abs=True)
				in_max_spd = False

			else:
				# Fast move
				pos_regs = []
				for m_id, pos_angle in zip(time_target.ids, positions):
					pos_regs.append(self._chain.angle_to_pos_reg(m_id, pos_angle))

				if in_max_spd:
					self._chain.sync_write_pos(time_target.ids, pos_regs)
				else:
					self._chain.sync_write_pos_speed(time_target.ids, pos_regs, speeds_max)
					in_max_spd = True


			# Wait for next time
			next_time_abs = beg_time + time_tar
			print "next_time %f " % next_time_abs
			while self.default_timer() < next_time_abs:
				pass


	def traj_move(self, id, tar_list):
		"""
		:type tar_list: list[TargetInfo]
		"""

		if not tar_list:
			return
		last_tar = tar_list.pop()

		beg_time = self.default_timer()
		for target in tar_list:
			traj_time = self.default_timer() - beg_time
			self._move_to(id, target.pos, target.time, traj_time, move_abs=False)
			next_time = beg_time + target.time
			while self.default_timer() < next_time:
				pass

		traj_time = self.default_timer() - beg_time
		self._move_to(id, last_tar.pos, last_tar.time, traj_time, move_abs=True)

	def _move_to(self, id, pos, time, time_now, move_abs):
		current_pos = self._chain.get_pos_phy(id)
		pos_diff = pos - current_pos
		time_diff = time - time_now

		dps = 0
		action_pos = 0
		if time_diff <= 0:
			# too late for target, use max speed
			dps = 0
			action_pos = pos
		else:
			dps = abs(pos_diff) / time_diff
			if move_abs:
				action_pos = pos
			else:
				if pos_diff < 0:
					action_pos = max(pos - 10, -180)
				elif pos_diff > 0:
					action_pos = min(pos + 10, 180)
				else:
					action_pos = pos

		self._chain.goto_phy(id, action_pos, dps, blocking=False)

	# print "[%f] pos %f to %f, Goal time: %f, Pos: %f, Speed %f" % (
	#	traj_time_now, current_pos, target.pos, target.time, pos, dps)
	# print traj_time_now


if __name__ == '__main__':
	chain = DxlChain("COM3", 1000000)
	print chain.get_motor_list()
	traj_ctrl = TrajectoryControl(chain)

	angle_list = [
		-0.22509, -0.22448, -0.22386, -0.22324, -0.22262, -0.22200, -0.22138, -0.22075, -0.22013, -0.21950,
		-0.21887, -0.21824, -0.21760, -0.21697, -0.21633, -0.21569, -0.21505, -0.21441, -0.21377, -0.21312,
		-0.21248, -0.21183, -0.21118, -0.21052, -0.20987, -0.20921, -0.20856, -0.20790, -0.20724, -0.20657,
		-0.20591, -0.20524, -0.20458, -0.20391, -0.20323, -0.20256, -0.20189, -0.20121, -0.20053, -0.19985,
		-0.19917, -0.19849, -0.19780, -0.19712, -0.19643, -0.19574, -0.19505, -0.19435, -0.19366, -0.19296,
		-0.19226, -0.19156, -0.19086, -0.19016, -0.18945, -0.18875, -0.18804, -0.18733, -0.18662, -0.18590,
		-0.18519, -0.18447, -0.18375, -0.18303, -0.18231, -0.18159, -0.18086, -0.18013, -0.17941, -0.17868,
		-0.17794, -0.17721, -0.17647, -0.17574, -0.17500, -0.17426, -0.17352, -0.17277, -0.17203, -0.17128,
		-0.17053, -0.16978, -0.16903, -0.16828, -0.16752, -0.16676, -0.16601, -0.16525, -0.16448, -0.16372,
		-0.16295, -0.16219, -0.16142, -0.16065, -0.15988, -0.15910, -0.15833, -0.15755, -0.15677, -0.15599,
		-0.15521, -0.15443, -0.15364, -0.15286, -0.15207, -0.15128, -0.15049, -0.14969, -0.14890, -0.14810,
		-0.14731, -0.14651, -0.14570, -0.14490, -0.14410, -0.14329, -0.14248, -0.14167, -0.14086, -0.14005,
		-0.13923, -0.13842, -0.13760, -0.13678, -0.13596, -0.13514, -0.13431, -0.13349, -0.13266, -0.13183,
		-0.13100, -0.13017, -0.12933, -0.12850, -0.12766, -0.12682, -0.12598, -0.12514, -0.12430, -0.12345,
		-0.12261, -0.12176, -0.12091, -0.12005, -0.11920, -0.11835, -0.11749, -0.11663, -0.11577, -0.11491,
		-0.11405, -0.11318, -0.11232, -0.11145, -0.11058, -0.10971, -0.10884, -0.10796, -0.10709, -0.10621,
		-0.10533, -0.10445, -0.10357, -0.10268, -0.10180, -0.10091, -0.10002, -0.09913, -0.09824, -0.09735,
		-0.09645, -0.09555, -0.09466, -0.09376, -0.09285, -0.09195, -0.09105, -0.09014, -0.08923, -0.08832,
		-0.08741, -0.08650, -0.08558, -0.08467, -0.08375, -0.08283, -0.08191, -0.08099, -0.08007, -0.07914,
		-0.07821, -0.07728, -0.07635, -0.07542, -0.07449, -0.07355, -0.07262, -0.07168, -0.07074, -0.06980,
		-0.06885, -0.06791, -0.06696, -0.06602, -0.06507, -0.06412, -0.06316, -0.06221, -0.06125, -0.06030,
		-0.05934, -0.05838, -0.05741, -0.05645, -0.05549, -0.05452, -0.05355, -0.05258, -0.05161, -0.05064,
		-0.04966, -0.04868, -0.04771, -0.04673, -0.04575, -0.04476, -0.04378, -0.04279, -0.04181, -0.04082,
		-0.03983, -0.03883, -0.03784, -0.03684, -0.03585, -0.03485, -0.03385, -0.03285, -0.03184, -0.03084,
		-0.02983, -0.02882, -0.02781, -0.02680, -0.02579, -0.02477, -0.02376, -0.02274, -0.02172, -0.02070,
		-0.01968, -0.01865, -0.01763, -0.01660, -0.01557, -0.01454, -0.01351, -0.01247, -0.01144, -0.01040,
		-0.00936, -0.00832, -0.00728, -0.00624, -0.00519, -0.00415, -0.00310, -0.00205, -0.00100, 0.00006,
		0.00111, 0.00217, 0.00323, 0.00428, 0.00535, 0.00641, 0.00747, 0.00854, 0.00961, 0.01068,
		0.01175, 0.01282, 0.01389, 0.01497, 0.01605, 0.01713, 0.01821, 0.01929, 0.02037, 0.02146,
		0.02255, 0.02364, 0.02473, 0.02582, 0.02691, 0.02801, 0.02911, 0.03021, 0.03131
	]

	time_polit = 0.001

	if True:
		tar_list = []
		i = 1
		for angle in angle_list:
			if i % 10 == 0:
				tar_list.append(TargetInfo(
					pos=angle * 57.295779, time=i * time_polit))
			i += 1

		traj_ctrl.traj_move(1, tar_list)

	if False:
		time_target = TimeTarget()
		time_target.ids = [1 ,2 ]
		time_target.times.append(2)
		time_target.trajs.append([-180] * 2)
		time_target.times.append(3)
		time_target.trajs.append([0] * 2)
		time_target.times.append(4)
		time_target.trajs.append([180] * 2)
		traj_ctrl.multi_traj_move(time_target)

	if False:
		i = 1
		for angle in angle_list:
			if i % 10 == 0:
				pos = angle * 57.296779
				time_target.times.append(i * time_polit)
				time_target.trajs.append([pos] * 2)
			i += 1

		traj_ctrl.multi_traj_move(time_target)

