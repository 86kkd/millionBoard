#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import re
import argparse
import pandas as pd
from matplotlib.gridspec import GridSpec


def parse_log_file(filename):
    """Parse the ESP32 ADC calibrated voltage log file"""
    pattern = r"I \((\d+)\) EXAMPLE: ADC1 Calibrated Voltage \(mV\): \[0\]:(\d+), \[1\]:(\d+), \[2\]:(\d+), \[3\]:(\d+), \[4\]:(\d+), \[5\]:(\d+), \[6\]:(\d+), \[7\]:(\d+), \[8\]:(\d+), \[9\]:(\d+)"

    timestamps = []
    channel_data = {i: [] for i in range(10)}  # Store all 10 channels

    with open(filename, "r") as f:
        for line in f:
            match = re.match(pattern, line.strip())
            if match:
                timestamp = int(match.group(1))
                timestamps.append(timestamp)

                # Extract all channel values
                for i in range(10):
                    channel_data[i].append(int(match.group(i + 2)))

    # Convert to DataFrame for easier manipulation
    data = {"timestamp": timestamps}
    for i in range(10):
        data[f"ch{i}"] = channel_data[i]

    return pd.DataFrame(data)


def plot_channels(df, channels=range(3, 7)):
    """Plot specific channels from the DataFrame"""
    # Create a figure with subplots
    fig = plt.figure(figsize=(12, 10))

    # Normalize time to seconds from start
    start_time = df["timestamp"].min()
    df["time_sec"] = (df["timestamp"] - start_time) / 1000.0

    # Create a grid layout with a larger subplot for all channels together
    # and individual subplots for each channel
    gs = GridSpec(len(channels) + 1, 1, height_ratios=[2] + [1] * len(channels))

    # Plot all channels together
    ax_all = fig.add_subplot(gs[0])
    for ch in channels:
        ax_all.plot(df["time_sec"], df[f"ch{ch}"], label=f"Channel {ch}")
    ax_all.set_title("ESP32 ADC Channels 3-6 Voltage Measurements")
    ax_all.set_ylabel("Voltage (mV)")
    ax_all.grid(True)
    ax_all.legend()

    # Plot individual channels
    for i, ch in enumerate(channels):
        ax = fig.add_subplot(gs[i + 1], sharex=ax_all)
        ax.plot(df["time_sec"], df[f"ch{ch}"], label=f"Channel {ch}")
        ax.set_ylabel(f"CH{ch} (mV)")
        ax.set_title(f"Channel {ch}")
        ax.grid(True)

        # Add statistics
        if len(df) > 0:
            stats = (
                f"Mean: {df[f'ch{ch}'].mean():.1f} mV\n"
                f"Std: {df[f'ch{ch}'].std():.1f} mV\n"
                f"Min: {df[f'ch{ch}'].min()} mV\n"
                f"Max: {df[f'ch{ch}'].max()} mV"
            )
            ax.text(
                0.02,
                0.85,
                stats,
                transform=ax.transAxes,
                bbox=dict(boxstyle="round", alpha=0.2),
            )

    # Set the xlabel only on the bottom subplot
    ax.set_xlabel("Time (seconds)")

    plt.tight_layout()
    return fig


def analyze_correlations(df, channels=range(3, 7)):
    """Analyze correlations between channels"""
    # Create a correlation matrix for the selected channels
    correlation_data = {f"ch{ch}": df[f"ch{ch}"] for ch in channels}
    corr_df = pd.DataFrame(correlation_data)
    correlation = corr_df.corr()

    # Plot correlation matrix
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(correlation, cmap="coolwarm")

    # Add labels, ticks, and colorbar
    channel_labels = [f"Channel {ch}" for ch in channels]
    ax.set_xticks(np.arange(len(channels)))
    ax.set_yticks(np.arange(len(channels)))
    ax.set_xticklabels(channel_labels)
    ax.set_yticklabels(channel_labels)

    # Rotate x tick labels
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")

    # Add colorbar
    cbar = ax.figure.colorbar(im, ax=ax)

    # Loop over data dimensions and create text annotations
    for i in range(len(channels)):
        for j in range(len(channels)):
            text = ax.text(
                j,
                i,
                f"{correlation.iloc[i, j]:.2f}",
                ha="center",
                va="center",
                color="black",
            )

    ax.set_title("Correlation Matrix Between ADC Channels")
    plt.tight_layout()
    return fig


def plot_fft(df, channels=range(3, 7), sample_rate=None):
    """Plot frequency spectrum of the signals"""
    # If sample_rate is not provided, estimate it from timestamps
    if sample_rate is None:
        # Estimate sample rate from timestamps (in Hz)
        timestamps = df["timestamp"].values
        time_diffs = np.diff(timestamps)
        avg_time_diff_ms = np.mean(time_diffs)
        sample_rate = 1000.0 / avg_time_diff_ms  # Hz

    fig, axes = plt.subplots(len(channels), 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"Frequency Analysis (Sample Rate: {sample_rate:.1f} Hz)")

    for i, ch in enumerate(channels):
        data = df[f"ch{ch}"].values

        # Remove DC component (mean)
        data = data - np.mean(data)

        # Compute FFT
        fft_result = np.fft.rfft(data)
        freqs = np.fft.rfftfreq(len(data), 1 / sample_rate)

        # Plot only up to Nyquist frequency (sample_rate/2)
        nyquist_idx = (
            len(freqs) if len(freqs) < 100 else 100
        )  # Limit to first 100 points for visibility

        # Plot magnitude spectrum
        axes[i].plot(freqs[:nyquist_idx], np.abs(fft_result)[:nyquist_idx])
        axes[i].set_ylabel(f"CH{ch} Magnitude")
        axes[i].set_title(f"Channel {ch} Frequency Spectrum")
        axes[i].grid(True)

    axes[-1].set_xlabel("Frequency (Hz)")
    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="ESP32 ADC Data Analyzer")
    parser.add_argument("--file", default="data.csv", help="Log file with ADC data")
    parser.add_argument("--output", help="Output prefix for saving plots")
    args = parser.parse_args()

    # Parse the log file
    df = parse_log_file(args.file)

    if df.empty:
        print("No data found in the file. Check the file format.")
        return

    print(f"Found {len(df)} data points for 10 channels")

    # Create plots
    fig_timeseries = plot_channels(df)
    fig_correlation = analyze_correlations(df)
    fig_fft = plot_fft(df)

    # Save or show plots
    if args.output:
        fig_timeseries.savefig(f"{args.output}_timeseries.png")
        fig_correlation.savefig(f"{args.output}_correlation.png")
        fig_fft.savefig(f"{args.output}_fft.png")
        print(f"Plots saved with prefix: {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
