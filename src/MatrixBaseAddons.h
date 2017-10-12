
bool operator <(const Matrix &mat)
const {
  for (int i=0; i<mat.size(); i++)
    {
      if (this->operator()(i) < mat(i))
	{
	  return true;
	}
      else if (this->operator()(i) > mat(i))
	{
	  return false;
	}
    }
  return false;
};
