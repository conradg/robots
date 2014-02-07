import System.IO
import System.Environment
type Matrix a = (
                 (a, a)  
                ,(a, a)
                ) 
main = do 
        args <- getArgs
        let handle = case args of
                        [x] -> openFile x ReadMode
                        _   -> return stdin
        file <- handle >>= hGetContents
        let contents = map (\x -> case words x of 
                                    a:b:xs -> p (read a) (read b)
                                    _     -> error "invalid input format") $ lines file
        let (row1, row2) = covarianceMatrix contents 
        putStrLn $ show row1
        putStrLn $ show row2


matrix :: a -> a -> a -> a -> Matrix a
matrix x1 x2 x3 x4 = ((x1,x2),(x3,x4))


covarianceMatrix :: [(Double,Double)] -> Matrix Double
covarianceMatrix coords = matrix tl tr bl br
    where
    len = fromIntegral $length coords
    avg xory cs = (sum (map xory cs)) / len
    avgx = avg fst coords
    avgy = avg snd coords
    tl = (sum (map (\a -> ((fst a) - avgx)**2) coords)) /len
    br = (sum (map (\a -> ((snd a) - avgy)**2) coords)) /len
    tr = (sum (map (\a -> ((snd a) - avgy) * ((fst a) - avgx)) coords)) /len
    bl = tr

point :: Double -> Double -> (Double,Double)
point a b = (a,b)
p = point
