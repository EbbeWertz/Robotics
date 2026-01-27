import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from isfr_bot_msgs.msg import YoloVisionObjectArray

from collections import defaultdict, Counter
from dataclasses import dataclass
import time
import math


# =======================
# GRID CONFIG
# =======================
GRID_RESOLUTION = 0.5       # meters per cell
GRID_WIDTH = 100
GRID_HEIGHT = 100
GRID_ORIGIN_X = -25.0
GRID_ORIGIN_Y = -25.0


# =======================
# BELIEF STRUCTURE
# =======================
@dataclass
class SemanticBelief:
    mean_count: float = 0.0
    pos_conf: float = 0.0
    neg_conf: float = 0.0
    neg_mag: float = 0.0
    last_pos: float = 0.0
    last_neg: float = 0.0

# =======================
# NODE
# =======================
class KnowledgeMapNode(Node):

    def __init__(self):
        super().__init__('knowledge_map')

        self.belief_grid = defaultdict(
            lambda: defaultdict(
                lambda: defaultdict(SemanticBelief)
            )
        )

        self.subscription = self.create_subscription(
            YoloVisionObjectArray,
            '/vision/absolute_position',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/knowledge/occupancy',
            10
        )

        self.get_logger().info('Knowledge map gestart (Semantic Belief → OccupancyGrid)')


    # =======================
    # WORLD → CELL
    # =======================
    def world_to_cell(self, x, y):
        cx = int((x - GRID_ORIGIN_X) / GRID_RESOLUTION)
        cy = int((y - GRID_ORIGIN_Y) / GRID_RESOLUTION)
        return cx, cy


    # =======================
    # BELIEF UPDATE
    # =======================
    def update_belief(self, belief: SemanticBelief, observed_count: int, edge_factor: float):
        now = time.time()

        # Positief bewijs (sterk)
        if observed_count >= belief.mean_count:
            delta = observed_count - belief.mean_count
            belief.mean_count += 0.5 * delta
            belief.pos_conf = min(1.0, belief.pos_conf + 0.2)
            belief.last_pos = now

        # Negatief bewijs (zwak en asymmetrisch)
        else:
            diff = belief.mean_count - observed_count
            neg_weight = (0.05 * edge_factor) / (1.0 + diff)
            belief.neg_conf = min(1.0, belief.neg_conf + neg_weight)
            belief.neg_mag = max(belief.neg_mag, diff)
            belief.last_neg = now

        belief.mean_count = max(0.0, belief.mean_count)


    # =======================
    # CALLBACK
    # =======================
    def listener_callback(self, msg):

        # Observaties per cel per object type
        observations = defaultdict(Counter)

        for obj in msg.objects:
            cx, cy = self.world_to_cell(obj.x, obj.y)

            if 0 <= cx < GRID_WIDTH and 0 <= cy < GRID_HEIGHT:
                observations[(cx, cy)][obj.label] += 1

        # Update belief grid (alleen waar observaties zijn)
        for (cx, cy), counts in observations.items():
            for obj_type, count in counts.items():
                belief = self.belief_grid[cx][cy][obj_type]

                # edge_factor placeholder (1.0 = centrum FOV)
                edge_factor = 1.0
                self.update_belief(belief, count, edge_factor)

        # Publish OccupancyGrid
        grid_msg = self.build_occupancy_grid()
        self.publisher.publish(grid_msg)


    # =======================
    # BELIEF → OCCUPANCY
    # =======================
    def build_occupancy_grid(self):

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        grid.info.resolution = GRID_RESOLUTION
        grid.info.width = GRID_WIDTH
        grid.info.height = GRID_HEIGHT
        grid.info.origin.position.x = GRID_ORIGIN_X
        grid.info.origin.position.y = GRID_ORIGIN_Y
        grid.info.origin.orientation.w = 1.0

        data = [0] * (GRID_WIDTH * GRID_HEIGHT)

        for cx in range(GRID_WIDTH):
            for cy in range(GRID_HEIGHT):

                score = 0.0

                for belief in self.belief_grid[cx][cy].values():
                    score += belief.mean_count * belief.pos_conf
                    score -= belief.neg_conf * belief.neg_mag

                score = max(0.0, score)
                value = int(min(100, score * 20.0))

                idx = cy * GRID_WIDTH + cx
                data[idx] = value

        grid.data = data
        return grid


# =======================
# MAIN
# =======================
def main(args=None):
    rclpy.init(args=args)
    node = KnowledgeMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
