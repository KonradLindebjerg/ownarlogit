import argparse
import math
import statistics

import camera


def wrap_angle(angle):
	return (angle + math.pi) % (2.0 * math.pi) - math.pi


def read_float(prompt):
	while True:
		value = input(prompt).strip()
		if value.lower() == "q":
			return None
		try:
			return float(value)
		except ValueError:
			print("Enter a number, or q to finish calibration.")


def collect_pose_samples(cam, sample_count, true_distance_cm, true_bearing_deg,
						 marker_id):
	distance_errors = []
	bearing_errors = []
	attempts = 0
	max_attempts = sample_count * 10
	true_bearing_rad = math.radians(true_bearing_deg)

	while len(distance_errors) < sample_count and attempts < max_attempts:
		attempts += 1
		image = cam.get_next_frame()
		ids, distances_cm, bearings_rad = cam.detect_aruco_objects(image)
		if ids is None:
			continue

		matches = [i for i, detected_id in enumerate(ids)
				   if marker_id is None or int(detected_id) == marker_id]
		if not matches:
			continue

		closest = min(matches, key=lambda i: distances_cm[i])
		distance_errors.append(float(distances_cm[closest]) - true_distance_cm)
		bearing_errors.append(wrap_angle(float(bearings_rad[closest]) - true_bearing_rad))

	return distance_errors, bearing_errors, attempts


def circular_mean(angles):
	return math.atan2(sum(math.sin(angle) for angle in angles),
					  sum(math.cos(angle) for angle in angles))


def main():
	parser = argparse.ArgumentParser(
		description="Estimate ArUco camera distance and bearing measurement noise."
	)
	parser.add_argument("--camera-index", type=int, default=0)
	parser.add_argument(
		"--robot-type",
		choices=("arlo", "frindo", "scribbler", "macbookpro"),
		default="arlo",
	)
	parser.add_argument(
		"--marker-id",
		type=int,
		default=None,
		help="Only use this marker ID; by default, use the closest detected marker.",
	)
	parser.add_argument("--samples-per-pose", type=int, default=20)
	args = parser.parse_args()

	if args.samples_per_pose < 2:
		parser.error("--samples-per-pose must be at least 2")

	print("ArUco measurement-noise calibration")
	print("Measure from the camera lens to the marker center.")
	print("Bearing uses the camera convention: positive is left, in degrees.")
	print("Use several setups at different ranges and bearings for a useful estimate.")
	print("Enter q at the distance prompt when all setups are collected.\n")

	cam = None
	all_distance_errors = []
	all_bearing_errors = []
	try:
		cam = camera.Camera(args.camera_index, robottype=args.robot_type,
							useCaptureThread=False)

		pose_number = 0
		while True:
			pose_number += 1
			print(f"Setup {pose_number}: position the robot and marker, then measure ground truth.")
			true_distance_cm = read_float("True camera-to-marker distance (cm, or q to finish): ")
			if true_distance_cm is None:
				break
			if true_distance_cm <= 0.0:
				print("Distance must be greater than zero.\n")
				pose_number -= 1
				continue

			true_bearing_deg = read_float("True camera-relative bearing (degrees; left is positive): ")
			if true_bearing_deg is None:
				break

			input(f"Show marker to camera; press Enter to collect {args.samples_per_pose} detections...")
			distance_errors, bearing_errors, attempts = collect_pose_samples(
				cam,
				args.samples_per_pose,
				true_distance_cm,
				true_bearing_deg,
				args.marker_id,
			)
			all_distance_errors.extend(distance_errors)
			all_bearing_errors.extend(bearing_errors)
			print(f"Collected {len(distance_errors)}/{args.samples_per_pose} samples "
				  f"in {attempts} frames.")
			if distance_errors:
				print(f"  Mean distance residual: {statistics.mean(distance_errors):+.2f} cm")
				mean_bearing = circular_mean(bearing_errors)
				print(f"  Mean bearing residual: {math.degrees(mean_bearing):+.2f} deg")
			print()

		if len(all_distance_errors) < 2:
			print("Not enough detections to estimate standard deviations; collect at least 2.")
			return 1

		mean_distance_error = statistics.mean(all_distance_errors)
		sigma_distance_cm = statistics.stdev(all_distance_errors)

		mean_bearing_error = circular_mean(all_bearing_errors)
		centered_bearing_errors = [wrap_angle(error - mean_bearing_error)
								   for error in all_bearing_errors]
		sigma_bearing_rad = statistics.stdev(centered_bearing_errors)

		print("Calibration summary")
		print(f"  Samples: {len(all_distance_errors)}")
		print(f"  Distance bias: {mean_distance_error:+.2f} cm")
		print(f"  Bearing bias: {math.degrees(mean_bearing_error):+.2f} deg")
		print(f"  Distance sigma: {sigma_distance_cm:.2f} cm")
		print(f"  Bearing sigma: {sigma_bearing_rad:.5f} rad "
			  f"({math.degrees(sigma_bearing_rad):.2f} deg)")
		print("\nSuggested observation-model constants:")
		print(f"SIGMA_D = {sigma_distance_cm:.4f}  # cm")
		print(f"SIGMA_THETA = {sigma_bearing_rad:.6f}  # rad, camera bearing noise")
		print("Note: bearing sigma here measures camera noise, not robot turn-motion noise.")

		if abs(mean_distance_error) > sigma_distance_cm:
			print("Warning: distance bias exceeds one sigma; correct camera/range bias "
				  "before relying on this sigma.")
		if abs(mean_bearing_error) > sigma_bearing_rad:
			print("Warning: bearing bias exceeds one sigma; check camera alignment "
				  "before relying on this sigma.")
		return 0
	except KeyboardInterrupt:
		print("\nCalibration interrupted.")
		return 130
	finally:
		if cam is not None:
			cam.terminateCaptureThread()


if __name__ == "__main__":
	raise SystemExit(main())
