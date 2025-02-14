import 'package:bookly/Features/home/data/models/book_model/book_model.dart';
import 'package:bookly/Features/home/presentation/views/widgets/custom_book_image.dart';
import 'package:bookly/constants.dart';
import 'package:bookly/core/utils/app_router.dart';
import 'package:bookly/core/utils/styles.dart';
import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';
import 'book_rating.dart';

class BestSellerItem extends StatelessWidget {
  const BestSellerItem({super.key, required this.bookModel});
  final BookModel bookModel;
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(
        vertical: 10,
      ),
      child: GestureDetector(
        onTap: () {
          GoRouter.of(context).push(
            AppRouter.kBookDetailsView,
          );
        },
        child: SizedBox(
          height: 125,
          child: Padding(
            padding: const EdgeInsets.symmetric(
              horizontal: 20,
            ),
            child: Row(
              children: [
                CustomBookImage(
                    imageUrl: bookModel.volumeInfo.imageLinks?.thumbnail ?? ''),
                const SizedBox(
                  width: 30,
                ),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const SizedBox(
                        height: 6,
                      ),
                      SizedBox(
                        width: MediaQuery.of(context).size.width * 0.6,
                        child: Text(
                          bookModel.volumeInfo.title!,
                          style: Styles.textStyle20.copyWith(
                            fontFamily: kGtSektraFine,
                          ),
                          maxLines: 2,
                          overflow: TextOverflow.ellipsis,
                        ),
                      ),
                      const SizedBox(
                        height: 3,
                      ),
                      Text(
                        bookModel.volumeInfo.authors![0],
                        overflow: TextOverflow.ellipsis,
                        style: Styles.textStyle14,
                      ),
                      const SizedBox(
                        height: 3,
                      ),
                      Row(
                        children: [
                          Text(
                            'Free',
                            style: Styles.textStyle20.copyWith(
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          const Spacer(),
                          const BookRating(rating: 4.8, count: (500)),
                        ],
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
