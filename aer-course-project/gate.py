import math
class Gate() :
    def __init__(self, x, y, z, phi, delta) :
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.calc_entry_exit_gates(delta)
    def calc_entry_exit_gates(self, delta) :
        if self.phi != 0 :
            self.entry = [self.x + delta, self.y]
            self.exit = [self.x - delta, self.y]
            self.entry2 = [self.x, self.y + 1.5*delta]
            self.exit2 = [self.x, self.y - 1.5*delta]
            self.obs_1 = [self.x, self.y + 0.2125]
            self.obs_2 = [self.x, self.y - 0.2125]
            self.entry11 = [self.x + delta, self.y - 0.2125]
            self.entry12 = [self.x + delta, self.y + 0.2125]
        else :
            self.entry = [self.x, self.y + delta]
            self.exit = [self.x, self.y - delta]
            self.entry2 = [self.x + 1.5*delta, self.y]
            self.exit2 = [self.x - 1.5*delta, self.y]
            self.obs_1 = [self.x + 0.2125, self.y]
            self.obs_2 = [self.x - 0.2125, self.y]
            self.entry11 = [self.x + 0.2125, self.y + delta]
            self.entry12 = [self.x - 0.2125, self.y + delta]
    def calc_sequence(self, SourceX, SourceY) :
        Dist1 = math.sqrt((self.entry[0] - SourceX)**2 + (self.entry[1] - SourceY)**2)
        Dist2 = math.sqrt((self.exit[0] - SourceX)**2 + (self.exit[1] - SourceY)**2)

        if Dist1 < Dist2 :
            return [[self.entry[0], self.entry[1]], [self.x, self.y], [self.exit[0], self.exit[1]], [self.entry2[0], self.entry2[1]], [self.exit2[0], self.exit2[1]], [self.entry11[0], self.entry11[1]], [self.entry12[0], self.entry12[1]]]
        else :
            return [[self.exit[0], self.exit[1]], [self.x, self.y], [self.entry[0], self.entry[1]], [self.entry2[0], self.entry2[1]], [self.exit2[0], self.exit2[1]], [self.entry11[0], self.entry11[1]], [self.entry12[0], self.entry12[1]]]

