def get_intervals(arr, f, shift=0):
    """Get the intervals in the list that satisfies a specific function
    
    Arguments:
        arr {list} -- The list to operate on
        f {function} -- The function that returns a boolen value given a list
        shift {int} -- Shift in returned indice
    
    Returns:
        [list] -- List of list (not tuple) containing intervals of staring index and ending index
    """
    prev_status = False
    status = False
    intervals = []
    i_start = -1
    for (i,element) in enumerate(arr):
        i += shift
        isSatisfied = f(element)
        if isSatisfied:
            status = True
            if not prev_status: # Prev status is false
                i_start = i
        else:
            status = False
            if prev_status:
                i_end = i
                intervals.append([i_start, i_end])
        prev_status = status

    if prev_status: # Last element
        intervals.append([i_start, len(arr)+shift])
    
    return intervals

def non_maximum_suppression(arr, intervals):
    """Suppress intervals to the center element
    
    Arguments:
        arr {original array of elements} -- Original array of Elements
        intervals {list of tuples} -- Interval, given by get_intervals
    
    Returns:
        list -- List of center elements
    """

    return [arr[(s+e)//2] for s,e in intervals]

def dilate(intervals, length):
    """Dilate the 1d intervals by length
    
    Arguments:
        intervals {2d list} -- Intervals got fromget_intervals, note it will be modified in place
        length {int} -- length for dilation
    
    Returns:
        2d list -- dilated intervals
    """

    for i in xrange(len(intervals)):
        intervals[i][0] -= length
        intervals[i][1] += length
    for i in xrange(len(intervals)-1):
        if (intervals[i][1] >= intervals[i+1][0]):
            intervals[i+1][0] = intervals[i][0]
            intervals[i] = None
    return [x for x in intervals if x is not None]

def filter_noise(intervals, length):
    """Dilate the 1d intervals by length
    
    Arguments:
        intervals {2d list} -- Intervals got fromget_intervals, note it will be modified in place
        length {int} -- maximum length that will be filtered out
    
    Returns:
        2d list -- filtered intervals
    """

    return [x for x in intervals if x[1]-x[0] > length]