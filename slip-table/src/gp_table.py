from .slipslope_db import SlipSlopeCsvDB
import pandas as pd
from .rksml_interp import load_rksml_file
from scipy.spatial.transform import Rotation as R
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import numpy as np
from sklearn.gaussian_process.kernels import WhiteKernel
import matplotlib.pyplot as plt
from sklearn.gaussian_process.kernels import DotProduct
from scipy import stats
import os
import glob

class SlipSlopeTable:
    def __init__(self,db_filename):
        self.db = SlipSlopeCsvDB(None)
        return
    
    # theta is 5-dim parameter set
    # d is distance in metric space
    def fetch_datapoints(self,theta,d):
        v = self.db.query(theta,d)
        return v
    
    def add_datapoints_slip_table(self,theta,slip_slope_dir):
        pattern = os.path.join(slip_slope_dir, "slope_*.csv")
        slip_files = glob.glob(pattern)
        
        # print(slip_files)
        combined_data = []
        
        for f in slip_files:
            filename = os.path.basename(f)  
            slope = float(filename[6:-4]) 
            slip_df = pd.read_csv(f)
           
            s_sim = slip_df.slow_slip.to_numpy(dtype=float)
            valid_sim = ~np.isnan(s_sim)
            valid_idx = np.where(valid_sim)[0]
            # interp_idx = np.arange(len(s_sim))
            # s_sim[~valid_sim] = np.interp(interp_idx[~valid_sim], valid_idx, s_sim[valid_sim])
         
            s_sim = s_sim[valid_idx]
            
            # mean_slip = np.mean(s_sim[2:])
           
            # print(s_sim)
            
             
            mean_slip = np.mean(s_sim[1:])
            print(mean_slip)

            
            # print(mean_slip)
            
            combined_data.append({
                'slip': mean_slip,
                'pitch': slope  
            })
        
        if combined_data:
            slip_slope_df = pd.DataFrame(combined_data)
            self.db.add(theta, slip_slope_df)
        else:
            print(f"No valid slip files found in {slip_slope_dir}")
        
        return  
        
    def add_datapoints(self,theta,rksml_file,slip_incons):
        res = load_rksml_file(rksml_file)
            
        data = res['data']
        data = data[data['SCLK'] > res['range'][0]]
        data = data[data['SCLK'] < res['range'][1]]

        data = data[['SCLK', 'QUAT_X', 'QUAT_Y', 'QUAT_Z', 'QUAT_C']]
        
        slip_df = pd.read_csv(slip_incons)

        slip_df = slip_df[['#sclk0_0','fast_slip_frac']]       
        slip_df = slip_df.rename(columns = { "#sclk0_0" : "SCLK", "fast_slip_frac" : "slip"})
       
        rotations = data[['QUAT_X','QUAT_Y','QUAT_Z','QUAT_C']]
        r = R.from_quat(rotations).as_euler(seq="XYZ",degrees=True).T
        
        data["pitch"] = -r[0]
        data = data[["SCLK","pitch"]]
        
        slip_df = slip_df.dropna()
        slip_df = pd.merge_asof(slip_df, data, on='SCLK', direction='nearest')
        self.db.add(theta, slip_df)
        
    def fit_gp(self,theta, d):
        v = self.db.query(theta,d).metadata_df
        slip_slope_tab = [[0,0]]
        
        for i in range(len(v)):
            v[i] = v[i][['slip','pitch']].to_numpy()
            slip_slope_tab = np.vstack([slip_slope_tab,v[i]])
       
        # print(slip_slope_tab)
        slip_slope_tab = slip_slope_tab[1:]
        
        poly_kernel = DotProduct(sigma_0=0.1)**1.5

        # Combined with RBF for flexibility
        kernel = poly_kernel + RBF(length_scale = 0.1) + WhiteKernel(noise_level=1.0)
        # range_x = 1.0
        # k_long = Matern(length_scale=max(range_x*0.5, 1.0), nu=2.5)
        # k_multi = RationalQuadratic(length_scale=max(range_x*0.2, 0.5), alpha=1.0)
        # kernel = C(1.0, (1e-3, 1e3)) * (k_long + k_multi) + WhiteKernel(noise_level=0.1, noise_level_bounds=(1e-6, 1e1))

        # kernel = C() * RBF() + WhiteKernel()        
        gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
        
        
        # print(slip_slope_tab.T)
        
        X_train = slip_slope_tab.T[1]
        y_train = slip_slope_tab.T[0]
        
        gp.fit(X_train.reshape(-1,1),y_train)
        
 
        # Make predictions
        X_test = np.linspace(0, 30, 100).reshape(-1, 1)
        y_pred, y_std = gp.predict(X_test, return_std=True)
        
        
        # Plot results
        plt.figure(figsize=(10, 6))
        plt.plot(X_test, y_pred, 'b-', label='GP mean')
        plt.fill_between(X_test.ravel(), 
                        y_pred - 1.96*y_std, 
                        y_pred + 1.96*y_std,
                        alpha=0.3, color='gray', label='95% confidence',clip_on=False)
                
        alphas = [ 0.25, 0.5, 0.75, 1.0 ]
       
        for alpha in alphas:
            # compute CVaR from gp joint distribution
            z_alpha = stats.norm.ppf(alpha) 
            phi_z_alpha = stats.norm.pdf(z_alpha) 
            
            cvar = y_pred + (y_std * phi_z_alpha / alpha)
            plt.plot(X_test,cvar, label=f"CVaR alpha={alpha*100.}%")

        plt.scatter(X_train, y_train, c='red', s=50, zorder=10, label='RKSML')
        plt.xlabel('Pitch [Deg]')
        plt.ylabel('Slip Fraction [-]')
        plt.title('Slip Model')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()

def main():
           
         
    table = SlipSlopeTable(None)
    # table.add_datapoints([5,4,3,2,1], "../downlink/sol01294_playback_vo_corrected.rksml","../downlink/sol01294_slip_incons.txt")
    #table.add_datapoints([5,4,3,2,1], "downlink/sol01590_playback_vo_corrected.rksml","downlink/sol01590_slip_incons.txt")
    # table.add_datapoints([5,4,3,2,1], "../downlink/sol01578_playback_vo_corrected.rksml.xml","../downlink/sol01578_slip_incons.txt")
    # table.add_datapoints([10,3,3,2,1], "../downlink/sol01286_playback_vo_corrected.rksml","../downlink/sol01286_slip_incons.txt")

    table.add_datapoints_slip_table([5,4,3,2,1],os.getcwd())
    # print(table.db.df)
    # print(v)

    table.fit_gp([1,2,3,4,5],20)
