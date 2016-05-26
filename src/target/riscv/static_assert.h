/** @file

@copyright Syntacore
*/
#ifndef STATIC_ASSERT_H_
#define STATIC_ASSERT_H_

#define STATIC_ASSERT2(COND,LINE) enum {static_assertion_at_line_##LINE= 1 / !!(COND)}
#define STATIC_ASSERT1(COND,LINE) STATIC_ASSERT2(COND,LINE)
#define STATIC_ASSERT(COND)  STATIC_ASSERT1(COND,__LINE__)

#endif  // STATIC_ASSERT_H_
