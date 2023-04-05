from typing import Tuple
import numpy as np
from numpy.typing import NDArray
from torch import Tensor
import torchvision.transforms.functional as TF
import torchvision.transforms as T
from torch.utils.data import Dataset


class GraspDataset(Dataset):
    def __init__(self, train: bool=True) -> None:
        '''Dataset of successful grasps.  Each data point includes a 64x64
        top-down RGB image of the scene and a grasp pose specified by the gripper
        position in pixel space and rotation (either 0 deg or 90 deg)

        The datasets are already created for you, although you can checkout
        `collect_dataset.py` to see how it was made (this can take a while if you
        dont have a powerful CPU).
        '''
        mode = 'train' if train else 'val'
        self.train = train
        data = np.load(f'{mode}_dataset.npz')
        self.imgs = data['imgs']
        self.actions = data['actions']

    def transform_grasp(self, img: Tensor, action: np.ndarray) -> Tuple[Tensor, np.ndarray]:
        '''Randomly rotate grasp by 0, 90, 180, or 270 degrees.  The image can be
        rotated using `TF.rotate`, but you will have to do some math to figure out
        how the pixel location and gripper rotation should be changed.

        Arguments
        ---------
        img:
            float tensor ranging from 0 to 1, shape=(3, 64, 64)
        action:
            array containing (px, py, rot_id), where px specifies the row in
            the image (heigh dimension), py specifies the column in the image (width dimension),
            and rot_id is an integer: 0 means 0deg gripper rotation, 1 means 90deg rotation.

        Returns
        -------
        tuple of (img, action) where both have been transformed by random
        rotation in the set (0 deg, 90 deg, 180 deg, 270 deg)

        Note
        ----
        The gripper is symmetric about 180 degree rotations so a 180deg rotation of
        the gripper is equivalent to a 0deg rotation and 270 deg is equivalent to 90 deg.

        Example Action Rotations
        ------------------------
        action = (42, 42, 0)
        - Rot   0 deg : rot_action = (42, 42, 0)
        - Rot  90 deg : rot_action = (21, 42, 1)
        - Rot 180 deg : rot_action = (21, 21, 0)
        - Rot 270 deg : rot_action = (42, 21, 1)

        action = (15, 45, 1)
        - Rot   0 deg : rot_action = (15, 45, 1)
        - Rot  90 deg : rot_action = (18, 15, 0)
        - Rot 180 deg : rot_action = (48, 18, 1)
        - Rot 270 deg : rot_action = (45, 48, 0)

    
        '''
        angle_choices = [0, 90, 180, 270]
        angle = int(np.random.choice(angle_choices))
        img = TF.rotate(img, angle)
        px, py, rot_id = action

        img_dim = 63  # The maximum index for the 64x64 image

        if angle == 90:
            px, py = img_dim - py, px
            rot_id = (rot_id + 1) % 2
        elif angle == 180:
            px, py = img_dim - px, img_dim - py
        elif angle == 270:
            px, py = py, img_dim - px
            rot_id = (rot_id + 1) % 2

        action = np.array([px, py, rot_id])
        
        ################################
        # Implement this function for Q4
        ################################
        return img, action

    def __getitem__(self, idx: int) -> Tuple[Tensor, Tensor]:
        img = self.imgs[idx]
        action = self.actions[idx]

        H, W = img.shape[:2]
        img = TF.to_tensor(img)
        if np.random.rand() < 0.5:
            img = TF.rgb_to_grayscale(img, num_output_channels=3)

        if self.train:
            img, action = self.transform_grasp(img, action)

        px, py, rot_id = action
        label = np.ravel_multi_index((rot_id, px, py), (2, H, W))

        return img, label

    def __len__(self) -> int:
        '''Number of grasps within dataset'''
        return self.imgs.shape[0]

# if __name__ == "__main__":
#     # Create dataset instance
#     dataset = GraspDataset(train=True)

#     # Get a sample image from the dataset
#     img, _ = dataset.__getitem__(0)  # You can change the index to any valid index in the dataset

#     # Test action rotations
#     action = np.array([15, 45, 1])
#     rotation_angles = [0, 90, 180, 270]
#     rotated_img, rotated_action = dataset.transform_grasp(img, action)

#     print(f"Original action: {action}")
#     print(f"Rotated action: {rotated_action}")