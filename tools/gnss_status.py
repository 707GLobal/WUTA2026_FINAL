"""CGI GNSS codes from msg_interfaces/Hcinspvatzcb.msg."""
GNSS_STATUS = {
    0: '不定位不定向', 1: '单点定位定向', 2: '伪距差分定位定向', 3: '组合推算',
    4: 'RTK固定解定位定向', 5: 'RTK浮点解定位定向', 6: '单点定位不定向',
    7: '伪距差分定位不定向', 8: 'RTK固定解定位不定向', 9: 'RTK浮点解定位不定向',
}


def gnss_description(code):
    return GNSS_STATUS.get(code, '未知状态')


def gnss_allows_start(code):
    return code == 4
