"""
SP3 File Interpolation Script
Interpolates SP3 satellite orbit data to 1-second intervals and outputs a new SP3 file
"""

import numpy as np
import pandas as pd
from datetime import datetime, timedelta
import time
import os

# ============================================================================
# CONFIGURATION - HARDCODED FILE PATHS
# ============================================================================
# Replace these with your actual SP3 file paths
YESTERDAY_SP3 = "COD0OPSFIN_20241850000_01D_05M_ORB.SP3"  # e.g., "gfz21234.sp3"
TODAY_SP3 = "COD0OPSFIN_20241860000_01D_05M_ORB.SP3"          # e.g., "gfz21235.sp3"
TOMORROW_SP3 = "COD0OPSFIN_20241870000_01D_05M_ORB.SP3"    # e.g., "gfz21236.sp3"
CLOCK_FILE = "path/to/clock.clk"         # e.g., "gfz21235.clk"
START_TIMESTAMP = datetime(2024, 7, 4, 6, 0, 0)   # Start time
END_TIMESTAMP = datetime(2024, 7, 4, 7, 0, 0)  # End time
OUTPUT_SP3 = "interpolated_1sec.sp3"
INTERPOLATION_INTERVAL = 5  # seconds
POLYNOMIAL_DEGREE = 16

# ============================================================================
# SP3 FILE READING FUNCTIONS
# ============================================================================

def read_sp3_file(filename):
    """
    Read SP3 file and return DataFrame with satellite positions
    """
    print(f"Reading SP3 file: {filename}")
    
    epochs = []
    satellites = []
    x_coords = []
    y_coords = []
    z_coords = []
    clocks = []
    
    with open(filename, 'r') as f:
        current_epoch = None
        
        for line in f:
            # Epoch line
            if line.startswith('*'):
                year = int(line[3:7])
                month = int(line[8:10])
                day = int(line[11:13])
                hour = int(line[14:16])
                minute = int(line[17:19])
                second = float(line[20:31])
                current_epoch = pd.Timestamp(datetime(year, month, day, hour, minute, int(second)))
            
            # Position line
            elif line.startswith('P'):
                if current_epoch is None:
                    continue
                
                sv = line[1:4].strip()
                x = float(line[4:18])
                y = float(line[18:32])
                z = float(line[32:46])
                clock = float(line[46:60]) if len(line) > 46 else 0.0
                
                epochs.append(current_epoch)
                satellites.append(sv)
                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)
                clocks.append(clock)
    
    df = pd.DataFrame({
        'Epoch': epochs,
        'SV': satellites,
        'X': x_coords,
        'Y': y_coords,
        'Z': z_coords,
        'Clock': clocks
    })
    
    df['deltaT'] = df['Epoch'].diff().dt.total_seconds()
    df = df.set_index(['Epoch', 'SV'])
    
    print(f"  Found {len(df)} data points")
    return df


def read_clock_file(filename):
    """
    Read clock file and return DataFrame
    """
    print(f"Reading clock file: {filename}")
    
    if not os.path.exists(filename):
        print(f"  Warning: Clock file not found. Creating empty clock data.")
        return pd.DataFrame(columns=['SV', 'Epoch', 'Clock', 'ClockRate'])
    
    epochs = []
    satellites = []
    clocks = []
    clock_rates = []
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('AS'):
                parts = line.split()
                sv = parts[1]
                year = int(parts[2])
                month = int(parts[3])
                day = int(parts[4])
                hour = int(parts[5])
                minute = int(parts[6])
                second = float(parts[7])
                
                epoch = pd.Timestamp(datetime(year, month, day, hour, minute, int(second)))
                clock = float(parts[9]) if len(parts) > 9 else 0.0
                clock_rate = float(parts[10]) if len(parts) > 10 else 0.0
                
                epochs.append(epoch)
                satellites.append(sv)
                clocks.append(clock)
                clock_rates.append(clock_rate)
    
    df = pd.DataFrame({
        'SV': satellites,
        'Epoch': epochs,
        'Clock': clocks,
        'ClockRate': clock_rates
    })
    
    print(f"  Found {len(df)} clock entries")
    return df


# ============================================================================
# INTERPOLATION FUNCTIONS
# ============================================================================

def coord_interp(poly_coeffs, interval):
    """
    Interpolate coordinates using polynomial coefficients
    """
    # time_points = np.arange(0, 10800, interval)  # 3 hours in seconds
    # poly = np.poly1d(poly_coeffs)
    # return poly(time_points)

    epoch = np.linspace(1800, 12600 , int(10800/interval)+1) # 3h validity interval within 4h
    time = np.array([epoch**deg for deg in range(len(poly_coeffs)-1,-1,-1)])
    return np.matmul(poly_coeffs,time)


def sp3_interpolation(epoch, interval=1, poly_degree=16):
    """
    Main interpolation function
    
    Args:
        epoch: datetime object for the target day
        interval: interpolation interval in seconds (default: 1)
        poly_degree: polynomial degree for fitting (default: 16)
    
    Returns:
        DataFrame with interpolated satellite positions and velocities
    """
    epoch_yesterday = epoch - timedelta(days=1)
    epoch_tomorrow = epoch + timedelta(days=1)
    
    # Read SP3 files
    yes = read_sp3_file(YESTERDAY_SP3)
    tod = read_sp3_file(TODAY_SP3)
    tom = read_sp3_file(TOMORROW_SP3)
    #clock = read_clock_file(CLOCK_FILE)
    clock = []
    
    # Validate polynomial degree
    if poly_degree > 16:
        raise Warning("Polynomial degree above 16 is not applicable!")
    elif poly_degree < 11:
        print("Warning: Polynomial degree below 11 is not recommended!")
    
    start = time.time()
    
    # Drop rows with missing deltaT
    yes = yes.dropna(subset=["deltaT"])
    tod = tod.dropna(subset=["deltaT"])
    tom = tom.dropna(subset=["deltaT"])
    
    # Reorder levels
    yes = yes.reorder_levels(["Epoch", "SV"])
    tod = tod.reorder_levels(["Epoch", "SV"])
    tom = tom.reorder_levels(["Epoch", "SV"])
    
    # Select time windows
    yes = yes.loc[(slice(
        pd.Timestamp(datetime(epoch_yesterday.year, epoch_yesterday.month, epoch_yesterday.day, 23, 0, 0)),
        pd.Timestamp(datetime(epoch_yesterday.year, epoch_yesterday.month, epoch_yesterday.day, 23, 59, 59))
    ))]
    
    tom = tom.loc[(slice(
        pd.Timestamp(datetime(epoch_tomorrow.year, epoch_tomorrow.month, epoch_tomorrow.day, 0, 0, 0)),
        pd.Timestamp(datetime(epoch_tomorrow.year, epoch_tomorrow.month, epoch_tomorrow.day, 3, 0, 0))
    ))]
    
    # Concatenate all SP3 data
    sp3 = pd.concat([yes, tod, tom], axis=0)
    svList = sp3.index.get_level_values("SV").unique()

    svList = svList.sort_values()
    svList = svList[svList.str.startswith('G')]
    print(f"\nInterpolating data for {len(svList)} satellites")
    print(f"Satellites: {', '.join(svList)}")
    
    # Get time information
    epoch_values = sp3.index.get_level_values("Epoch").unique()
    deltaT = epoch_values[1] - epoch_values[0]
    fitTime = np.linspace(0, deltaT.seconds * 16, 17)
    header = ['X', 'Y', 'Z', 'Vx', 'Vy', 'Vz']
    
    # Create interpolation time range
    epoch_start = pd.Timestamp(datetime(epoch_yesterday.year, epoch_yesterday.month, epoch_yesterday.day, 23, 0, 0))
    epoch_step = timedelta(hours=3)
    epoch_stop = epoch_start + timedelta(hours=4)
    
    dti = pd.date_range(
        start=pd.Timestamp(datetime(epoch_yesterday.year, epoch_yesterday.month, epoch_yesterday.day, 23, 30, 0)),
        end=pd.Timestamp(datetime(epoch_tomorrow.year, epoch_tomorrow.month, epoch_tomorrow.day, 2, 29, 59)),
        freq=str(interval) + 'S'
    )
    
    # Create output dataframe
    index = pd.MultiIndex.from_product([svList, dti.tolist()], names=['SV', 'Epoch'])
    interp_coord = pd.DataFrame(index=index, columns=header)
    interp_coord = interp_coord.reorder_levels(['Epoch', 'SV'])
    interp_coord = interp_coord.sort_index()
    
    # Interpolation loop
    points_per_window = int(10800 / interval)
    
    while True:
        sp3_temp = sp3.loc[(slice(epoch_start, epoch_stop))].copy()
        sp3_temp = sp3_temp.reorder_levels(["SV", "Epoch"])
        
        epoch_interp_List = np.zeros(shape=(points_per_window, 6, len(svList)))
        
        for svIndex, sv in enumerate(svList):
            epoch_number = len(sp3_temp.loc[sv])
            
            if epoch_number <= poly_degree:
                print(f"Warning: Not enough epochs for {sv} | Epoch Count: {epoch_number} - Poly Degree: {poly_degree}")
                epoch_interp_List[:, :, svIndex] = np.full(shape=(points_per_window, 6), fill_value=np.nan)
                continue
            
            if epoch_number != 17:
                fitTime = [(sp3_temp.loc[sv].index[t] - sp3_temp.loc[sv].index[0]).seconds for t in range(epoch_number)]
            
            # Fit sp3 coordinates to polynomial
            fitX = np.polyfit(fitTime, sp3_temp.loc[sv].X.copy(), deg=poly_degree)
            fitY = np.polyfit(fitTime, sp3_temp.loc[sv].Y.copy(), deg=poly_degree)
            fitZ = np.polyfit(fitTime, sp3_temp.loc[sv].Z.copy(), deg=poly_degree)
            
            # Interpolate coordinates
            x_interp = coord_interp(fitX, interval) * 1000  # km to m
            x_velocity = np.array([(x_interp[i+1] - x_interp[i]) / interval if (i+1) < len(x_interp) else 0 for i in range(len(x_interp))])
            
            y_interp = coord_interp(fitY, interval) * 1000  # km to m
            y_velocity = np.array([(y_interp[i+1] - y_interp[i]) / interval if (i+1) < len(y_interp) else 0 for i in range(len(y_interp))])
            
            z_interp = coord_interp(fitZ, interval) * 1000  # km to m
            z_velocity = np.array([(z_interp[i+1] - z_interp[i]) / interval if (i+1) < len(z_interp) else 0 for i in range(len(z_interp))])
            
            sv_interp = np.vstack((x_interp[:-1], y_interp[:-1], z_interp[:-1], x_velocity[:-1], y_velocity[:-1], z_velocity[:-1])).transpose()
            #sv_interp = np.vstack((x_interp, y_interp, z_interp, x_velocity, y_velocity, z_velocity)).transpose()

            epoch_interp_List[:, :, svIndex] = sv_interp
            
            fitTime = np.linspace(0, deltaT.seconds * 16, 17)  # restore original fitTime
        
        interp_coord.loc[(slice(epoch_start + timedelta(minutes=30), epoch_stop - timedelta(minutes=30, seconds=1))), ('X', 'Y', 'Z', 'Vx', 'Vy', 'Vz')] = epoch_interp_List.transpose(1, 0, 2).reshape(6, -1).transpose()
        
        epoch_start += epoch_step
        epoch_stop += epoch_step
        
        if epoch_start == pd.Timestamp(datetime(epoch_tomorrow.year, epoch_tomorrow.month, epoch_tomorrow.day, 2, 0, 0)):
            break
    
    # Merge with clock data
    interp_coord = interp_coord.reorder_levels(['Epoch', 'SV']).astype(float)
    
    # if not clock.empty:
    #     clock.index.name = 'SV'
    #     clock.set_index('Epoch', append=True, inplace=True)
    #     clock = clock.reorder_levels(['Epoch', 'SV'])
    #     epochMatch = interp_coord.index.intersection(clock.index)
    #     ephemerisMatched = interp_coord.loc[epochMatch]
    #     clockMatched = clock.loc[epochMatch]
    #     sp3matched = pd.concat([ephemerisMatched, clockMatched], axis=1)
    # else:
    sp3matched = interp_coord
    sp3matched['Clock'] = 0.0
    sp3matched['ClockRate'] = 0.0
    
    finish = time.time()
    print(f"\nSP3 interpolation completed in {finish-start:.2f} seconds")
    
    return sp3matched


# ============================================================================
# SP3 FILE WRITING FUNCTION
# ============================================================================

def write_sp3_file(df, filename, epoch):
    """
    Write interpolated data to SP3 format file
    Only saves epochs between START_TIMESTAMP and END_TIMESTAMP
    
    Args:
        df: DataFrame with interpolated positions
        filename: output filename
        epoch: reference epoch for the file
    """
    print(f"\nWriting SP3 file: {filename}")
    print(f"Filtering epochs between {START_TIMESTAMP} and {END_TIMESTAMP}")
    
    # Filter dataframe to only include epochs within the specified range
    df_reordered = df.reorder_levels(['Epoch', 'SV']).sort_index()
    
    # Convert timestamps to pandas Timestamp for comparison
    start_ts = pd.Timestamp(START_TIMESTAMP)
    end_ts = pd.Timestamp(END_TIMESTAMP)
    
    # Filter by epoch range
    df_filtered = df_reordered.loc[(slice(start_ts, end_ts)), :]
    
    if len(df_filtered) == 0:
        print("Warning: No data found in the specified time range!")
        return
    
    num_epochs = len(df_filtered.index.get_level_values('Epoch').unique())
    print(f"Found {num_epochs} epochs in specified range")
    
    with open(filename, 'w') as f:
        # Write header
        f.write("#dP2023  1 27  0  0  0.00000000      86400 ORBIT IGS14 HLM  IGS\n")
        f.write("## 2028 345600.00000000   900.00000000 60480 0.0000000000000\n")
        
        # Get satellite list
        svList = df_filtered.index.get_level_values('SV').unique()
        
        # Write satellite list
        num_sats = len(svList)
        f.write(f"+   {num_sats:2d}   ")
        for i, sv in enumerate(svList):
            if i > 0 and i % 17 == 0:
                f.write("\n+        ")
            f.write(f"{sv:3s}")
        f.write("\n")
        
        # Write accuracy lines (placeholder)
        f.write("++       ")
        for i in range(min(17, num_sats)):
            f.write("  0")
        f.write("\n")
        
        # Write coordinate system
        f.write("%c M  cc GPS ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n")
        f.write("%c cc cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n")
        f.write("%f  1.2500000  1.025000000  0.00000000000  0.000000000000000\n")
        f.write("%f  0.0000000  0.000000000  0.00000000000  0.000000000000000\n")
        f.write("%i    0    0    0    0      0      0      0      0         0\n")
        f.write("%i    0    0    0    0      0      0      0      0         0\n")
        f.write("/* INTERPOLATED TO 1 SECOND INTERVAL\n")
        f.write("/* CREATED BY SP3 INTERPOLATION SCRIPT\n")
        f.write(f"/* FILTERED FROM {START_TIMESTAMP} TO {END_TIMESTAMP}\n")
        f.write("/* \n")
        
        # Write data
        current_epoch = None
        
        for idx, row in df_filtered.iterrows():
            epoch_time = idx[0]
            sv = idx[1]
            
            # Write epoch header if changed
            if current_epoch != epoch_time:
                current_epoch = epoch_time
                dt = current_epoch.to_pydatetime()
                f.write(f"*  {dt.year:4d} {dt.month:2d} {dt.day:2d} {dt.hour:2d} {dt.minute:2d} {dt.second:11.8f}\n")
            
            # Write position line (convert m to km)
            x = row['X'] / 1000.0 if not pd.isna(row['X']) else 0.0
            y = row['Y'] / 1000.0 if not pd.isna(row['Y']) else 0.0
            z = row['Z'] / 1000.0 if not pd.isna(row['Z']) else 0.0
            clock = row.get('Clock', 0.0) if 'Clock' in row and not pd.isna(row.get('Clock', 0.0)) else 0.0
            
            f.write(f"P{sv:3s}{x:14.6f}{y:14.6f}{z:14.6f}{clock:14.6f}\n")
        
        # Write EOF
        f.write("EOF\n")
    
    print(f"SP3 file written successfully with {len(df_filtered)} data points")
    print(f"Time range: {df_filtered.index.get_level_values('Epoch').min()} to {df_filtered.index.get_level_values('Epoch').max()}")




# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """
    Main execution function
    """
    print("="*80)
    print("SP3 INTERPOLATION TO 1-SECOND INTERVALS")
    print("="*80)
    
    # Set the target epoch (today)
    # target_epoch = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)
    # Or use a specific date:
    target_epoch = datetime(2024, 7, 4)
    
    print(f"\nTarget epoch: {target_epoch.strftime('%Y-%m-%d')}")
    print(f"Interpolation interval: {INTERPOLATION_INTERVAL} second(s)")
    print(f"Polynomial degree: {POLYNOMIAL_DEGREE}")
    
    # Perform interpolation
    interpolated_data = sp3_interpolation(
        target_epoch,
        interval=INTERPOLATION_INTERVAL,
        poly_degree=POLYNOMIAL_DEGREE
    )
    
    # Write output SP3 file
    write_sp3_file(interpolated_data, OUTPUT_SP3, target_epoch)
    
    print("\n" + "="*80)
    print("INTERPOLATION COMPLETE")
    print("="*80)
    print(f"Output file: {OUTPUT_SP3}")
    print(f"Total interpolated epochs: {len(interpolated_data.index.get_level_values('Epoch').unique())}")
    print(f"Total satellites: {len(interpolated_data.index.get_level_values('SV').unique())}")


if __name__ == "__main__":
    main()
