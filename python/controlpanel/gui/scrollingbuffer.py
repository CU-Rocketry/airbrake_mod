import numpy as np

class ScrollingBuffer:
    def __init__(self, max_size=1000, width=1):
        self.max_size = max_size
        self.width = width
        self.x = np.zeros(max_size, dtype=np.float64)        
        self.y = np.zeros((max_size, width), dtype=np.float64) # y may have multiple cols
            
        self.offset = 0
        self.size = 0

    def add_point(self, x, y):
        if self.size < self.max_size:
            self.x[self.size] = x
            self.y[self.size] = y 
            self.size += 1
        else:
            self.x[self.offset] = x
            self.y[self.offset] = y
            self.offset = (self.offset + 1) % self.max_size

    def get_data(self, col=0):
        # 1. Catch the None trap just in case your plots.py is passing it
        if col is None:
            col = 0
            
        # 2. Slice the arrays
        if self.size < self.max_size:
            x_raw = self.x[:self.size]
            y_raw = self.y[:self.size, col] # Slice the column out right here
        else:
            x_raw = np.concatenate((self.x[self.offset:], self.x[:self.offset]))
            # Concatenate the 1D slices, NOT the 2D matrices
            y_raw = np.concatenate((self.y[self.offset:, col], self.y[:self.offset, col]))
            
        # 3. .flatten() guarantees a 100% contiguous, 1D block of fresh memory.
        # This completely strips away any 2D memory formatting that ImPlot hates.
        return x_raw.flatten(), y_raw.flatten()
    
    def get_latest(self):
        if self.size == 0:
            return np.zeros(self.width, dtype=np.float64)
        
        # offset points to the NEXT index to overwrite, so the newest is right behind it
        idx = (self.offset - 1) % self.max_size if self.size == self.max_size else self.size - 1
        return self.y[idx, :]