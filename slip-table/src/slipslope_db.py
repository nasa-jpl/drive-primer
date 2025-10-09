import numpy as np
import pandas as pd
from scipy.spatial.distance import cdist

class SlipSlopeCsvDB:
    def __init__(self, csv_filename=None):
        if(csv_filename is not None):
            self.df = pd.read_csv(csv_filename)        
        else:
            self.df = pd.DataFrame(columns=["bulk_density","cohesion","friction","youngs_modulus","poisson_ratio","metadata_df"])
    
    def add(self, vector, df):
        self.df.loc[len(self.df)] = [vector[0],vector[1],vector[2],vector[3],vector[4], df]
        
    
    # return dataframe's with associated vectors within $d$ distance
    def query(self, vector, d=10):
        if(len(self.df.bulk_density) == 0):
            print("Warning: no datapoints initialized")
            return -1
        
        vectors = np.vstack(self.df[["bulk_density","cohesion","friction","youngs_modulus","poisson_ratio"]].values)
        query_vector = np.array(vector).reshape(1, -1)
        distances = cdist(query_vector, vectors, metric='euclidean')[0]
        _df = self.df.copy()
        _df['distances'] = distances
        _df = _df[_df['distances'] <= d]
        
        return _df 
    
    # nop for csv
    def build(self):
        return
    
    def save(self, path):
        pd.write_csv(path)
    
    def load(self, path):
        pass
    
# ss = SlipSlopeCsvDB()
# ss.add([1,2,3,4,5],pd.DataFrame([1,2,3]))
# ss.add([1,5,10,4,5],pd.DataFrame())
# ss.add([1,2,1,1,1],pd.DataFrame())

# q = ss.query([3,2,1,4,5],5)

# print(q['distances'])
