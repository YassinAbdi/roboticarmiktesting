class Solution(object):
    # If divisible by 3, print Fizz
    # If divisible by 5, print Buzz
    # If divisible by both, print FizzBuzz 
    # If neither, just print the number

    def fizzBuzz(self, n):
        """
        :type n: int
        :rtype: List[str]
        """
        result = []
        for i in range(1, n + 1):
            if i % 3 == 0 and i % 5 == 0:
                result.append("FizzBuzz")
            elif i % 3 == 0:
                result.append("Fizz")
            elif i % 5 == 0:
                result.append("Buzz")
            else:
                result.append(str(i))
        return result


if __name__ == "__main__":
    n = 15
    s = Solution()
    print(s.fizzBuzz(n))
