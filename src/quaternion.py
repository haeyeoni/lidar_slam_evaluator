import numpy as np


class Quaternion:
    def __init__(self, q):
        self.x = q[0]
        self.y = q[1]
        self.z = q[2]
        self.w = q[3]

    def __repr__(self):
        return "quaternion {} {}i {}j {}k".format(self.w, self.x, self.y, self.z)

    def __str__(self):
        return "quaternion {} {}i {}j {}k".format(self.w, self.x, self.y, self.z)

    def __mul__(self, other):
        return Quaternion([self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
                           self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
                           self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
                           self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z])

    def __pow__(self, power, modulo=None):
        if power == -1:
            return Quaternion([-self.x, -self.y, -self.z, self.w])

        norm = self.norm()
        if norm == 0:
            return Quaternion([self.x, self.y, self.z, self.w])

        norm_v = np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
        if 0.99 <= norm <= 1:
            phi = np.arccos(self.w)
            w = np.cos(power * phi)
            x = np.sin(power * phi) * self.x / norm_v
            y = np.sin(power * phi) * self.y / norm_v
            z = np.sin(power * phi) * self.z / norm_v
        else:
            mul = norm ** power
            phi = np.arccos(self.w / norm)
            w = mul * np.cos(power * phi)
            x = mul * np.sin(power * phi) * self.x / norm_v
            y = mul * np.sin(power * phi) * self.y / norm_v
            z = mul * np.sin(power * phi) * self.z / norm_v
        return Quaternion([x, y, z, w])

    def rotation(self):
        return np.array([[2 * (self.w ** 2 + self.x ** 2) - 1, 2 * (self.x * self.y - self.w * self.z), 2 * (self.x * self.z + self.w * self.y)],
                         [2 * (self.x * self.y + self.w * self.z), 2 * (self.w ** 2 + self.y ** 2) - 1, 2 * (self.y * self.z - self.w * self.x)],
                         [2 * (self.x * self.z - self.w * self.y), 2 * (self.y * self.z + self.w * self.x), 2 * (self.w ** 2 + self.z ** 2) - 1]])

    def norm(self):
        return np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2)


def SLERP(q1, q2, t):
    return q1 * ((q1 ** -1) * q2) ** t
