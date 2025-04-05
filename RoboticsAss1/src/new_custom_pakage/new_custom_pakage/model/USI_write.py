class USI_write:
    """
    Il model USI_write associa le posizioni (coordinate) alle lettere da disegnare.
    """
    def __init__(self):
        self.positions = {
            'U': (2.0, 5.5),
            'S': (6.0, 5.5),
            'I': (8.0, 5.5)
        }
        
        self.point = {
            'U': [(1, 7), (1, 3), (3, 3), (3, 7)],
            'S': [(6, 7), (4, 7), (4, 5), (6, 5), (6, 3), (4, 3)],
            'I': [(8, 7), (8, 3)]
        }
