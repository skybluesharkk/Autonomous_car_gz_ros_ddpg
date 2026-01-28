import rclpy
import torch
import numpy as np
import random
import argparse
from torch.utils.tensorboard import SummaryWriter

from env_node import EnvNode
from ddpg_agent import DDPGAgent
from frame_stack import FrameStack


def main():
    # 1. Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    seed = args.seed

    # 2. Seed
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    # 3. ROS2 init
    rclpy.init()

    writer = SummaryWriter(f"runs/ddpg_gazebo_ros2/seed_{seed}")

    env = EnvNode()
    agent = DDPGAgent()
    frame_stack = FrameStack(4)

    print(f"🚀 Start training (seed={seed})")

    for i_episode in range(1, env.MAX_EPISODE + 1):

        # ✅ episode 시작에서만 reset
        env.reset()
        obs_raw = env.get_observation()
        print("sensors min/max:", 
        np.min(obs_raw["sensors"]),
        np.max(obs_raw["sensors"]))
        print("has NaN:", np.isnan(obs_raw["sensors"]).any())
        state = frame_stack.reset(obs_raw)

        total_reward = 0.0
        episode_actor_loss = []
        episode_critic_loss = []
        episode_q_val = []

        for t in range(env.MAX_STEP):

            action = agent.get_action(state)

            next_obs_raw, reward, done = env.step(action)

            next_state = frame_stack.step(next_obs_raw)

            agent.replay_buffer.push(
                state,
                action,
                reward,
                next_state,
                float(done)
            )

            total_reward += reward

            # 학습
            if agent.replay_buffer.cnt > agent.batch_size:
                actor_loss, critic_loss, q_val = agent.train_model()
                if actor_loss is not None:
                    episode_actor_loss.append(actor_loss)
                if critic_loss is not None:
                    episode_critic_loss.append(critic_loss)
                if q_val is not None:
                    episode_q_val.append(q_val)

            # ✅ terminal이면 즉시 종료
            if done:
                env.publish_zero_action()
                rclpy.spin_once(env, timeout_sec=0.05)
                break

            state = next_state
            print(f"step {t} reward={total_reward:.2f} don={done}")
        # logging
        writer.add_scalar("Episode/Reward", total_reward, i_episode)
        writer.add_scalar("Episode/Steps", t + 1, i_episode)
        writer.add_scalar("Episode/Actor_Loss",
                          np.mean(episode_actor_loss) if episode_actor_loss else 0.0,
                          i_episode)
        writer.add_scalar("Episode/Critic_Loss",
                          np.mean(episode_critic_loss) if episode_critic_loss else 0.0,
                          i_episode)
        writer.add_scalar("Episode/Avg_Q_Value",
                          np.mean(episode_q_val) if episode_q_val else 0.0,
                          i_episode)

        print(f"[Episode {i_episode}] reward={total_reward:.2f}, steps={t+1}")

        if i_episode % 50 == 0:
            agent.record_check_point(
                path=f"models/ddpg_seed_{seed}",
                episode=i_episode
            )

    print("✅ Training complete")

    agent.save_model(path=f"models/ddpg_final_seed_{seed}")
    writer.close()

    env.destroy_node()
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
