#!/usr/bin/env python3
import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import LineCollection


def create_plots(df, axes):
    ax1, ax2, ax3 = axes
    ax1.clear()
    ax2.clear()
    ax3.clear()

    if df.empty or 'time' not in df.columns:
        return

    group_id = (df['time'].diff() < 0).cumsum()
    
    # Only keep flights with more than 10 data points to avoid plotting incomplete flights
    valid_flights = []
    for _, group in df.groupby(group_id):
        if len(group) > 10:
            valid_flights.append(group.copy())
            
    if valid_flights:
        for i in range(1, len(valid_flights)):
            offset = valid_flights[i-1]['time'].iloc[-1]
            valid_flights[i]['time'] += offset
        df = pd.concat(valid_flights, ignore_index=True)
    else:
        df = pd.DataFrame(columns=df.columns)

    t = df['time']

    ax1.plot(t, df['tgt_x'], 'r--', label='Target X')
    ax1.plot(t, df['rpos_x'], 'r-', label='Pos X')
    ax1.plot(t, df['tgt_y'], 'g--', label='Target Y')
    ax1.plot(t, df['rpos_y'], 'g-', label='Pos Y')
    ax1.plot(t, df['tgt_z'], 'b--', label='Target Z')
    ax1.plot(t, df['rpos_z'], 'b-', label='Pos Z')
    ax1.set_ylabel('Position (m)')
    ax1.legend(loc='upper left', fontsize='small', ncol=3)
    ax1.grid(True)

    ax2.plot(t, df['err_x'], 'r-', label='Err X')
    ax2.plot(t, df['err_y'], 'g-', label='Err Y')
    ax2.plot(t, df['err_z'], 'b-', label='Err Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.legend(loc='upper left', fontsize='small', ncol=3)
    ax2.grid(True)

    if 'vx' in df.columns and 'vy' in df.columns and 'vz' in df.columns:
        speed = np.sqrt(df['vx']**2 + df['vy']**2 + df['vz']**2)
    else:
        dt = df['time'].diff().replace(0, np.nan)
        vx = df['rpos_x'].diff() / dt
        vy = df['rpos_y'].diff() / dt
        vz = df['rpos_z'].diff() / dt
        speed = np.sqrt(vx**2 + vy**2 + vz**2).bfill().fillna(0)
    
    df['speed'] = speed

    if 'mode' in df.columns:
        df_fig8 = df[df['mode'] == 2].copy()
    else:
        df_fig8 = pd.DataFrame()

    if not df_fig8.empty:
        tgt_cx = (df_fig8['tgt_x'].max() + df_fig8['tgt_x'].min()) / 2.0
        tgt_cy = (df_fig8['tgt_y'].max() + df_fig8['tgt_y'].min()) / 2.0

        rpos_cx = (df_fig8['rpos_x'].max() + df_fig8['rpos_x'].min()) / 2.0
        rpos_cy = (df_fig8['rpos_y'].max() + df_fig8['rpos_y'].min()) / 2.0

        df_fig8['tgt_x'] += (rpos_cx - tgt_cx)
        df_fig8['tgt_y'] += (rpos_cy - tgt_cy)

        ax3.plot(df_fig8['tgt_x'], df_fig8['tgt_y'], color='black', linestyle='--', linewidth=1.5, label='2D Target', zorder=1)

        x = df_fig8['rpos_x'].values
        y = df_fig8['rpos_y'].values
        s = df_fig8['speed'].values
        
        if len(x) > 1:
            points = np.array([x, y]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            norm = plt.Normalize(s.min(), s.max())
            lc = LineCollection(segments, cmap='viridis', norm=norm, zorder=2)
            lc.set_array(s[:-1])
            lc.set_linewidth(2)
            ax3.add_collection(lc)

        ax3.set_aspect('equal', adjustable='datalim') 
        ax3.legend(loc='upper left', fontsize='small')
    else:
        ax3.text(0.5, 0.5, 'Waiting Figure-Eight mode', 
                 horizontalalignment='center', verticalalignment='center', transform=ax3.transAxes,
                 fontsize=12, color='gray')
        ax3.set_xticks([])
        ax3.set_yticks([])

    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.grid(True, linestyle='-', alpha=0.5)
    
    ax1.set_title('Position')
    ax2.set_title('Position Error')
    ax3.set_title('X-Y Trajectory')
    ax1.set_ylabel('Position (m)')
    ax2.set_ylabel('Error (m)')
    ax3.set_ylabel('Y (m)')
    ax2.set_xlabel('Time (s)')
    ax3.set_xlabel('X (m)')

def main():
    parser = argparse.ArgumentParser(description='Crazyflie flight log plotter')

    parser.add_argument('csv_file', help='Path to CSV file to plot')

    # delete the --live if you don't want real-time updating
    parser.add_argument('--live', action='store_true', help='Update plot in real time')
    
    args = parser.parse_args()

    if not os.path.exists(args.csv_file) and not args.live:
        print(f"Err: File {args.csv_file} not found.")
        sys.exit(1)

    fig = plt.figure(figsize=(10, 10))
    fig.canvas.manager.set_window_title('ANN Flight Data')

    filename = os.path.basename(args.csv_file)
    fig.suptitle(f'{filename}', fontsize=14)
    
    ax1 = fig.add_subplot(3, 1, 1)
    ax2 = fig.add_subplot(3, 1, 2, sharex=ax1)
    ax3 = fig.add_subplot(3, 1, 3)           
    
    axes = (ax1, ax2, ax3)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    if args.live:
        def update(frame):
            try:
                if os.path.exists(args.csv_file) and os.path.getsize(args.csv_file) > 0:
                    df = pd.read_csv(args.csv_file)
                    create_plots(df, axes)
            except Exception:
                pass 
        
        ani = animation.FuncAnimation(fig, update, interval=200, cache_frame_data=False)
        plt.show()
    else:
        df = pd.read_csv(args.csv_file)
        create_plots(df, axes)
        plt.show()

if __name__ == '__main__':
    main()