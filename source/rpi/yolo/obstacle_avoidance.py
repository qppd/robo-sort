"""
Obstacle Avoidance Module for RoboSort
Implements left vs right avoidance logic using LIDAR angle and distance data
"""

import math
from typing import Dict, Tuple, Optional


class ObstacleAvoidance:
    """
    Handles obstacle detection and avoidance decision making using LIDAR data
    """
    
    def __init__(
        self,
        safe_distance: float = 50.0,  # cm - minimum safe distance from obstacles
        critical_distance: float = 30.0,  # cm - critical distance requiring immediate action
        danger_distance: float = 15.0,  # cm - danger zone, must stop/back up
        front_angle_range: int = 30,  # degrees - front detection zone (±30° from center)
        left_angle_range: Tuple[int, int] = (30, 100),  # degrees - left detection zone
        right_angle_range: Tuple[int, int] = (260, 330),  # degrees - right detection zone
        side_weight: float = 0.7,  # Weight for side obstacle scoring
        clear_path_threshold: float = 80.0,  # cm - distance considered clear path
        valid_angle_ranges: list = None,  # List of (min, max) angle ranges to use
    ):
        """
        Initialize obstacle avoidance system
        
        Args:
            safe_distance: Minimum distance to maintain from obstacles (cm)
            critical_distance: Distance requiring immediate turning (cm)
            danger_distance: Distance requiring stop/reverse (cm)
            front_angle_range: Angular width of front detection zone (degrees)
            left_angle_range: Tuple (start, end) angles for left side (degrees)
            right_angle_range: Tuple (start, end) angles for right side (degrees)
            side_weight: Weight multiplier for side obstacle importance (0-1)
            clear_path_threshold: Distance considered as clear path (cm)
            valid_angle_ranges: List of (min, max) angle ranges to use (None = use all)
        """
        self.safe_distance = safe_distance
        self.critical_distance = critical_distance
        self.danger_distance = danger_distance
        self.front_angle_range = front_angle_range
        self.left_angle_range = left_angle_range
        self.right_angle_range = right_angle_range
        self.side_weight = side_weight
        self.clear_path_threshold = clear_path_threshold
        # Default to 0-100 and 260-360 if not specified
        self.valid_angle_ranges = valid_angle_ranges if valid_angle_ranges is not None else [(0, 100), (260, 360)]
        
    def normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to 0-360 range
        
        Args:
            angle: Input angle in degrees
            
        Returns:
            Normalized angle in range [0, 360)
        """
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle
    
    def is_valid_angle(self, angle: float) -> bool:
        """
        Check if angle is in valid reading range (to filter out rear-mounted objects)
        
        Args:
            angle: Angle in degrees (0-360)
            
        Returns:
            True if angle is in valid range
        """
        angle = self.normalize_angle(angle)
        for min_angle, max_angle in self.valid_angle_ranges:
            if min_angle <= angle <= max_angle:
                return True
        return False
    
    def is_in_front(self, angle: float) -> bool:
        """
        Check if angle is in front detection zone
        
        Args:
            angle: Angle in degrees (0-360)
            
        Returns:
            True if angle is in front zone
        """
        angle = self.normalize_angle(angle)
        # Front zone is ±front_angle_range from 0° (forward)
        return angle <= self.front_angle_range or angle >= (360 - self.front_angle_range)
    
    def is_on_left(self, angle: float) -> bool:
        """
        Check if angle is in left detection zone
        
        Args:
            angle: Angle in degrees (0-360)
            
        Returns:
            True if angle is on left side
        """
        angle = self.normalize_angle(angle)
        return self.left_angle_range[0] <= angle <= self.left_angle_range[1]
    
    def is_on_right(self, angle: float) -> bool:
        """
        Check if angle is in right detection zone
        
        Args:
            angle: Angle in degrees (0-360)
            
        Returns:
            True if angle is on right side
        """
        angle = self.normalize_angle(angle)
        return self.right_angle_range[0] <= angle <= self.right_angle_range[1]
    
    def analyze_obstacles(self, distances: Dict[float, float]) -> Dict:
        """
        Analyze LIDAR distances to detect obstacles in different zones
        
        Args:
            distances: Dictionary mapping angles (degrees) to distances (cm)
            
        Returns:
            Dictionary containing obstacle analysis results:
                - front_min_distance: Closest obstacle in front
                - left_min_distance: Closest obstacle on left
                - right_min_distance: Closest obstacle on right
                - front_obstacles: Number of obstacles detected in front
                - left_obstacles: Number of obstacles detected on left
                - right_obstacles: Number of obstacles detected on right
                - left_clear_space: Average distance on left side
                - right_clear_space: Average distance on right side
        """
        front_distances = []
        left_distances = []
        right_distances = []
        
        # Categorize distances by zone
        for angle, distance in distances.items():
            # Filter out invalid readings (0 or very large values)
            if distance <= 0 or distance > 1000:
                continue
            
            # Filter out angles not in valid range (e.g., rear of robot)
            if not self.is_valid_angle(angle):
                continue
                
            if self.is_in_front(angle):
                front_distances.append(distance)
            elif self.is_on_left(angle):
                left_distances.append(distance)
            elif self.is_on_right(angle):
                right_distances.append(distance)
        
        # Calculate metrics for each zone
        analysis = {
            'front_min_distance': min(front_distances) if front_distances else float('inf'),
            'left_min_distance': min(left_distances) if left_distances else float('inf'),
            'right_min_distance': min(right_distances) if right_distances else float('inf'),
            'front_obstacles': len([d for d in front_distances if d < self.safe_distance]),
            'left_obstacles': len([d for d in left_distances if d < self.safe_distance]),
            'right_obstacles': len([d for d in right_distances if d < self.safe_distance]),
            'left_clear_space': sum(left_distances) / len(left_distances) if left_distances else 0,
            'right_clear_space': sum(right_distances) / len(right_distances) if right_distances else 0,
            'front_clear_count': len([d for d in front_distances if d > self.clear_path_threshold]),
            'left_clear_count': len([d for d in left_distances if d > self.clear_path_threshold]),
            'right_clear_count': len([d for d in right_distances if d > self.clear_path_threshold]),
        }
        
        return analysis
    
    def calculate_turn_score(self, analysis: Dict, direction: str) -> float:
        """
        Calculate a score for turning in a specific direction
        Higher score = better direction to turn
        
        Args:
            analysis: Obstacle analysis dictionary
            direction: 'left' or 'right'
            
        Returns:
            Score value (higher is better)
        """
        if direction == 'left':
            # Score based on clear space and number of clear readings
            space_score = analysis['left_clear_space']
            clear_score = analysis['left_clear_count'] * 10
            obstacle_penalty = analysis['left_obstacles'] * 20
        else:  # right
            space_score = analysis['right_clear_space']
            clear_score = analysis['right_clear_count'] * 10
            obstacle_penalty = analysis['right_obstacles'] * 20
        
        # Combine scores
        total_score = space_score + clear_score - obstacle_penalty
        
        return total_score
    
    def decide_action(self, distances: Dict[float, float]) -> Tuple[str, int, Optional[str]]:
        """
        Decide what action to take based on LIDAR data
        
        Args:
            distances: Dictionary mapping angles (degrees) to distances (cm)
            
        Returns:
            Tuple of (action, speed, info):
                action: 'forward', 'backward', 'turn_left', 'turn_right', 'rotate_left', 'rotate_right', 'stop'
                speed: Speed value (0-255)
                info: Additional information string for debugging
        """
        if not distances:
            return ('stop', 0, 'No LIDAR data available')
        
        # Analyze obstacles
        analysis = self.analyze_obstacles(distances)
        
        front_dist = analysis['front_min_distance']
        left_dist = analysis['left_min_distance']
        right_dist = analysis['right_min_distance']
        
        # DANGER ZONE - Immediate stop or reverse
        if front_dist < self.danger_distance:
            return ('backward', 150, f'DANGER! Front obstacle at {front_dist:.1f}cm')
        
        # CRITICAL ZONE - Must turn or rotate
        if front_dist < self.critical_distance:
            # Calculate which direction is better
            left_score = self.calculate_turn_score(analysis, 'left')
            right_score = self.calculate_turn_score(analysis, 'right')
            
            if left_score > right_score:
                return ('rotate_left', 180, f'Critical! Rotating left (score: L={left_score:.1f} R={right_score:.1f})')
            else:
                return ('rotate_right', 180, f'Critical! Rotating right (score: L={left_score:.1f} R={right_score:.1f})')
        
        # WARNING ZONE - Turn while moving
        if front_dist < self.safe_distance:
            left_score = self.calculate_turn_score(analysis, 'left')
            right_score = self.calculate_turn_score(analysis, 'right')
            
            if left_score > right_score:
                return ('turn_left', 120, f'Warning! Turning left (score: L={left_score:.1f} R={right_score:.1f})')
            else:
                return ('turn_right', 120, f'Warning! Turning right (score: L={left_score:.1f} R={right_score:.1f})')
        
        # Path partially clear but obstacles on sides
        if analysis['front_obstacles'] > 0:
            # Some obstacles in front but not critical
            left_score = self.calculate_turn_score(analysis, 'left')
            right_score = self.calculate_turn_score(analysis, 'right')
            
            # Gentle turning
            if left_score > right_score and left_dist > self.safe_distance:
                return ('turn_left', 150, f'Gentle left turn (scores: L={left_score:.1f} R={right_score:.1f})')
            elif right_dist > self.safe_distance:
                return ('turn_right', 150, f'Gentle right turn (scores: L={left_score:.1f} R={right_score:.1f})')
        
        # ALL CLEAR - Move forward
        return ('forward', 180, f'Path clear (front: {front_dist:.1f}cm, L: {left_dist:.1f}cm, R: {right_dist:.1f}cm)')
    
    def get_status_report(self, distances: Dict[float, float]) -> str:
        """
        Generate a detailed status report of current obstacle situation
        
        Args:
            distances: Dictionary mapping angles (degrees) to distances (cm)
            
        Returns:
            Formatted status string
        """
        analysis = self.analyze_obstacles(distances)
        
        report = "=== OBSTACLE AVOIDANCE STATUS ===\n"
        report += f"Front: {analysis['front_min_distance']:.1f}cm "
        report += f"({analysis['front_obstacles']} obstacles)\n"
        report += f"Left:  {analysis['left_min_distance']:.1f}cm "
        report += f"({analysis['left_obstacles']} obstacles, avg: {analysis['left_clear_space']:.1f}cm)\n"
        report += f"Right: {analysis['right_min_distance']:.1f}cm "
        report += f"({analysis['right_obstacles']} obstacles, avg: {analysis['right_clear_space']:.1f}cm)\n"
        
        action, speed, info = self.decide_action(distances)
        report += f"\nRecommended Action: {action.upper()} @ speed {speed}\n"
        report += f"Info: {info}\n"
        report += "================================="
        
        return report
