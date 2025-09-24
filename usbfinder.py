import subprocess
import glob
from typing import List, Dict


class USBTTYFinder:
    def __init__(self, device_pattern: str = "/dev/ttyUSB*"):
        """
        Initialize the finder with a glob pattern for devices.
        Default is '/dev/ttyUSB*'.
        """
        self.device_pattern = device_pattern

    def _get_device_products(self, dev: str) -> List[str]:
        """
        Get all ATTRS{product} values for a given device.
        """
        try:
            result = subprocess.run(
                ["udevadm", "info", "-a", "-n", dev],
                capture_output=True, text=True, check=True
            )
            products = []
            for line in result.stdout.splitlines():
                if "ATTRS{product}" in line:
                    product = line.split("==")[-1].strip().strip('"')
                    products.append(product)
            return products
        except subprocess.CalledProcessError:
            return []

    def find_by_product(self, product_name: str) -> List[str]:
        """
        Find device paths that match a given product name substring.
        """
        matches = []
        for dev in glob.glob(self.device_pattern):
            for product in self._get_device_products(dev):
                if product_name.lower() in product.lower():
                    matches.append(dev)
                    break
        return matches

    def find_multiple(self, product_names: List[str]) -> Dict[str, List[str]]:
        """
        Find devices for multiple product names.
        Returns a dict mapping product_name -> list of device paths.
        """
        results = {}
        for name in product_names:
            results[name] = self.find_by_product(name)
        return results


# Example usage:
if __name__ == "__main__":
    finder = USBTTYFinder()
    print(finder.find_by_product("Position Aligner"))          # ['/dev/ttyUSB1']
    print(finder.find_by_product("Brushed Motor Controller"))  # ['/dev/ttyUSB2']
    print(finder.find_multiple(["Position Aligner", "Brushed Motor Controller"]))
    # {'Position Aligner': ['/dev/ttyUSB1'], 'Brushed Motor Controller': ['/dev/ttyUSB2']}
