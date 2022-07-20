    """Module for optimization of spider's movement. 
    """

    class GravityCompensator:

        def getPositionOffsets(self, legsIds, rgValues, forces):
            maxOffset = 10
